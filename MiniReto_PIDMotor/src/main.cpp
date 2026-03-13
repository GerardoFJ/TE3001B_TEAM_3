/*
Minichallenge Team 3
Gerardo Fregoso Jiménez
Daniel Eduardo Hinojosa Alvarado
José Raúl Arredondo López
Luis Fernando Ruíz Neria

Clock-hour DC motor position controller via Micro-ROS.
Receives an hour (0–12) and drives the motor to that clock position using
a quadrature encoder for feedback and a PID controller.

Topics:
  /clock_hour     (std_msgs/Int32, subscriber) — desired hour [0, 12]
  /motor_position (std_msgs/Int32, publisher)  — current position in encoder ticks
*/

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

// ======== Micro-ROS Entity Declarations ========
rclc_support_t   support;
rclc_executor_t  executor;
rcl_allocator_t  allocator;
rcl_node_t       node;

rcl_subscription_t hour_subscriber;
rcl_publisher_t    position_publisher;
rcl_timer_t        pid_timer;

std_msgs__msg__Int32 hour_msg;
std_msgs__msg__Int32 position_msg;

// ======== Macro Definitions ========
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

// ======== Hardware Pin Definitions ========
#define IN1_PINA  18    // H-bridge direction A
#define IN1_PINB  19    // H-bridge direction B
#define PWM_PIN    5    // PWM output to H-bridge enable
#define ENC_A     25    // Encoder channel A (interrupt capable)
#define ENC_B     26    // Encoder channel B (interrupt capable)

#define PWM_RES   16    // 16-bit PWM resolution
#define PWM_FRQ  980    // PWM frequency (Hz)
#define PWM_CHNL   0    // LEDC channel
#define MAX_PWM  ((1 << PWM_RES) - 1)   // 65535

// ======== Motor / Encoder Parameters ========
#define TICKS_REV      2480    // Encoder ticks per full shaft revolution (quadrature ×4)

// ======== PID Parameters (tune on hardware) ========
//   With 16-bit PWM and TICKS_REV=606:
//   Max error ≈ 606 ticks → Kp*606 should approach MAX_PWM.
//   Starting values — adjust Kp/Ki/Kd to suit your motor.
const float Kp = 130.0f;
const float Ki =   0.5f;
const float Kd =  0.0f;
const float DT = 0.02f;               // PID period (s) — 50 Hz timer
const float INTEGRAL_LIMIT = 20000.0f;
const float POSITION_TOLERANCE = 3.0f; // ticks — stop band
const float DEADBAND_PWM = 2500.0f;    // minimum PWM that overcomes static friction

// ======== Encoder State ========
volatile long encoder_ticks = 0;

void IRAM_ATTR encoderISR_A() {
  // Full quadrature: compare A and B levels on every edge of A
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
    encoder_ticks++;
  } else {
    encoder_ticks--;
  }
}

void IRAM_ATTR encoderISR_B() {
  // Full quadrature: compare A and B levels on every edge of B (opposite sense)
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
    encoder_ticks--;
  } else {
    encoder_ticks++;
  }
}

// ======== PID State ========
volatile long target_ticks = 0;
float pid_integral  = 0.0f;
float pid_prev_error = 0.0f;

// ======== Connection State Machine ========
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();

// ======== Subscriber Callback: Receive desired hour ========
// Maps any non-negative hour → absolute encoder ticks.
// 12 hours = 1 full revolution = TICKS_REV ticks.
// Hour 13 = 1 full revolution + 1 hour, hour 24 = 2 full revolutions, etc.
// Negative values are clamped to 0 (home).
void hour_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int32_t hour = msg->data;

  if (hour < 0) hour = 0;

  // Convert hour → absolute target encoder ticks (multi-revolution)
  target_ticks = (long)((hour / 12.0f) * (float)TICKS_REV);
}

// ======== PID Timer Callback (50 Hz) ========
void pid_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  long current = encoder_ticks;            // atomic read (long, Xtensa 32-bit)
  float error = (float)(target_ticks - current);

  if (fabsf(error) < POSITION_TOLERANCE) {
    // Within tolerance — hold still and reset integrator
    ledcWrite(PWM_CHNL, 0);
    pid_integral   = 0.0f;
    pid_prev_error = 0.0f;
  } else {
    // PID computation
    pid_integral += error * DT;
    pid_integral   = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - pid_prev_error) / DT;
    pid_prev_error = error;

    float output = Kp * error + Ki * pid_integral + Kd * derivative;

    // Set direction via H-bridge
    if (output >= 0.0f) {
      digitalWrite(IN1_PINA, HIGH);
      digitalWrite(IN1_PINB, LOW);
    } else {
      digitalWrite(IN1_PINA, LOW);
      digitalWrite(IN1_PINB, HIGH);
    }

    // Apply deadband and clamp PWM
    float pwm_val = constrain(fabsf(output), DEADBAND_PWM, (float)MAX_PWM);
    ledcWrite(PWM_CHNL, (uint16_t)pwm_val);
  }

  // Publish current position in encoder ticks
  position_msg.data = (int32_t)current;
  RCSOFTCHECK(rcl_publish(&position_publisher, &position_msg, NULL));
}

// ======== Setup ========
void setup() {
  set_microros_transports();

  // Motor driver pins
  pinMode(IN1_PINA, OUTPUT);
  pinMode(IN1_PINB, OUTPUT);
  pinMode(PWM_PIN,  OUTPUT);
  digitalWrite(IN1_PINA, LOW);
  digitalWrite(IN1_PINB, LOW);

  // PWM
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);
  ledcWrite(PWM_CHNL, 0);

  // Encoder with pull-ups (open-collector encoders)
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR_B, CHANGE);

  state = WAITING_AGENT;
}

// ======== Main Loop ========
void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                  ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                  ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}

// ======== ROS 2 Entity Creation ========
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "clock_motor_controller", "", &support));

  // Subscriber: /clock_hour — desired hour (0–12)
  RCCHECK(rclc_subscription_init_default(
    &hour_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "clock_hour"));

  // Publisher: /motor_position — current encoder ticks
  RCCHECK(rclc_publisher_init_default(
    &position_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_position"));

  // Timer: PID loop at 50 Hz (20 ms period)
  RCCHECK(rclc_timer_init_default(&pid_timer, &support,
    RCL_MS_TO_NS(20), pid_callback));

  // Executor: 1 subscriber + 1 timer
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &hour_subscriber, &hour_msg, &hour_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &pid_timer));

  return true;
}

// ======== ROS 2 Entity Cleanup ========
void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&hour_subscriber, &node);
  rcl_publisher_fini(&position_publisher, &node);
  rcl_timer_fini(&pid_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
