#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// ── Motor Driver Pins (L298N) ─────────────────────────────────
#define PIN_IN1  19   // Direction A  (L298N IN1)
#define PIN_IN2  18   // Direction B  (L298N IN2)
#define PIN_PWM   5   // PWM Enable   (L298N ENA)

// ── LEDC PWM config ───────────────────────────────────────────
#define PWM_CHANNEL    0
#define PWM_FREQ       980   // Hz
#define PWM_RESOLUTION 8     // bits → duty cycle 0-255

// ── micro-ROS entities ────────────────────────────────────────
rcl_subscription_t subscriber;
std_msgs__msg__Int32 cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ── Error loop: blink LED on failure ─────────────────────────
void error_loop() {
    while (1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ── Motor drive helper ────────────────────────────────────────
// pwm_value ∈ [-255, 255]: sign = direction, |value| = duty cycle
void set_motor(int pwm_value) {
    int duty = abs(constrain(pwm_value, -255, 255));

    if (pwm_value > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
    } else if (pwm_value < 0) {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    } else {
        // Brake: both low → motor coasts to stop
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
    }

    ledcWrite(PWM_CHANNEL, duty);
}

// ── /cmd_pwm subscriber callback ─────────────────────────────
void cmd_pwm_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    set_motor(msg->data);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // Motor direction pins
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);

    // LEDC PWM
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);

    // Start stopped
    set_motor(0);

    // micro-ROS serial transport (USB → micro-ROS Agent on PC)
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/cmd_pwm"
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &subscriber, &cmd_msg,
        &cmd_pwm_callback, ON_NEW_DATA
    ));
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
