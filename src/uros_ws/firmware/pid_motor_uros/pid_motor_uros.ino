/*
  micro-ROS PID speed controller for DC motor with encoder

  Subscribes:
    /set_point      std_msgs/msg/Float32   (%)

  Publishes:
    /motor_output   std_msgs/msg/Float32   (% velocidad)
*/

#include <Arduino.h>
#include <math.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// =========================
// micro-ROS declarations
// =========================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t setpoint_subscriber;
rcl_publisher_t motor_output_publisher;

std_msgs__msg__Float32 setpoint_msg;
std_msgs__msg__Float32 motor_output_msg;

// =========================
// Error macros
// =========================
#define RCCHECK(fn)  { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static int64_t init = -1; \
  if (init == -1) init = uxr_millis(); \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// =========================
// State machine
// =========================
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// =========================
// Pin definitions
// =========================
#define ENC_A_GPIO 25
#define ENC_B_GPIO 26
#define LED_GPIO 2

#define IN1_GPIO 19
#define IN2_GPIO 18
#define ENA_PWM_GPIO 5

// =========================
// PWM configuration
// =========================
static const int PWM_CHANNEL = 0;
static const int PWM_FREQ_HZ = 980;
static const int PWM_RES_BITS = 10;
static const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

// =========================
// Motor parameters
// =========================
static const float MAX_RPM = 90.0f;
static const float PULSES_PER_REV = 620.0f;

volatile long encPulsesSigned = 0;

// =========================
// PID parameters
// =========================
static const float Ts = 0.01f;

float Kp = 1.2f;
float Ki = 2.0f;
float Kd = 0.02f;

float e_k = 0.0f;
float e_k_1 = 0.0f;
float e_k_2 = 0.0f;

float u_k = 0.0f;
float u_k_1 = 0.0f;

static const float UMIN = -100.0f;
static const float UMAX = 100.0f;

// =========================
// Measurements
// =========================
float rpmMeas = 0.0f;
float speedPct = 0.0f;
float refPct = 0.0f;

unsigned long tPid = 0;
unsigned long tRpm = 0;

static const unsigned long RPM_WINDOW_MS = 75;

// =========================
// Utility
// =========================
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// =========================
// Encoder ISR
// =========================
void IRAM_ATTR isrEncA() {
  if (digitalRead(ENC_B_GPIO) == HIGH)
    encPulsesSigned++;
  else
    encPulsesSigned--;
}

// =========================
// Motor control
// =========================
void applyMotorCommand(float u_percent) {

  float u_sat = clampf(u_percent, UMIN, UMAX);

  if (u_sat >= 0) {
    digitalWrite(IN1_GPIO, HIGH);
    digitalWrite(IN2_GPIO, LOW);
  } else {
    digitalWrite(IN1_GPIO, LOW);
    digitalWrite(IN2_GPIO, HIGH);
  }

  float mag = fabsf(u_sat);

  int duty = (int)((mag / 100.0f) * PWM_MAX);
  duty = constrain(duty, 0, PWM_MAX);

  ledcWrite(PWM_CHANNEL, duty);
}

// =========================
// micro-ROS callback
// =========================
void setpoint_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  refPct = clampf(msg->data, -100.0f, 100.0f);
}

// =========================
// Create entities
// =========================
bool create_entities() {

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "esp32_pid_motor_controller",
    "",
    &support));

  RCCHECK(rclc_subscription_init_default(
    &setpoint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/set_point"));

  RCCHECK(rclc_publisher_init_default(
    &motor_output_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/motor_output"));

  executor = rclc_executor_get_zero_initialized_executor();

  RCCHECK(rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &setpoint_subscriber,
    &setpoint_msg,
    &setpoint_callback,
    ON_NEW_DATA));

  return true;
}

// =========================
// Destroy entities
// =========================
void destroy_entities() {

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&setpoint_subscriber, &node);
  rcl_publisher_fini(&motor_output_publisher, &node);

  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// =========================
// Setup
// =========================
void setup() {

  set_microros_transports();

  pinMode(LED_GPIO, OUTPUT);

  pinMode(IN1_GPIO, OUTPUT);
  pinMode(IN2_GPIO, OUTPUT);
  pinMode(ENA_PWM_GPIO, OUTPUT);

  pinMode(ENC_A_GPIO, INPUT);
  pinMode(ENC_B_GPIO, INPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(ENA_PWM_GPIO, PWM_CHANNEL);

  attachInterrupt(digitalPinToInterrupt(ENC_A_GPIO), isrEncA, FALLING);

  tPid = millis();
  tRpm = millis();

  state = WAITING_AGENT;
}

// =========================
// Main loop
// =========================
void loop() {

  unsigned long now = millis();

  if (now - tRpm >= RPM_WINDOW_MS) {

    tRpm = now;

    noInterrupts();
    long pulses = encPulsesSigned;
    encPulsesSigned = 0;
    interrupts();

    rpmMeas = ((float)pulses / PULSES_PER_REV) * (60000.0f / RPM_WINDOW_MS);

    speedPct = 100.0f * (rpmMeas / MAX_RPM);
    speedPct = clampf(speedPct, -120.0f, 120.0f);
  }

  if (now - tPid >= (unsigned long)(Ts * 1000.0f)) {

    tPid = now;

    e_k = refPct - speedPct;

    float a0 = (Kp + Ki * Ts + (Kd / Ts));
    float a1 = (-Kp - 2.0f * (Kd / Ts));
    float a2 = (Kd / Ts);

    u_k = u_k_1 + a0 * e_k + a1 * e_k_1 + a2 * e_k_2;

    u_k = clampf(u_k, UMIN, UMAX);

    applyMotorCommand(u_k);

    u_k_1 = u_k;
    e_k_2 = e_k_1;
    e_k_1 = e_k;

    digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
  }

  switch (state) {

    case WAITING_AGENT:

      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;);

      break;

    case AGENT_AVAILABLE:

      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

      if (state == WAITING_AGENT)
        destroy_entities();

      break;

    case AGENT_CONNECTED:

      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

      motor_output_msg.data = speedPct;

      RCSOFTCHECK(rcl_publish(&motor_output_publisher, &motor_output_msg, NULL));

      break;

    case AGENT_DISCONNECTED:

      destroy_entities();
      applyMotorCommand(0);
      refPct = 0;
      state = WAITING_AGENT;

      break;
  }
}
