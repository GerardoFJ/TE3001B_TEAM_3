/*
Minichallenge Team 4
Gerardo Fregoso Jiménez
Daniel Eduardo Hinojosa Alvarado
José Raúl Arredondo López
Luis Fernando Ruíz Neria
*/
#include <Arduino.h>
#include <micro_ros_arduino.h>           // Micro-ROS library for ESP32
#include <rcl/rcl.h>                     // ROS 2 client library (Core)
#include <rcl/error_handling.h>          // ROS 2 error handling utilities
#include <rclc/rclc.h>                   // Micro-ROS client library
#include <rclc/executor.h>               // Executor tuuuo handle callbacks
#include <std_msgs/msg/int32.h>          // Message Type declaration
#include <rmw_microros/rmw_microros.h>   // Middleware functions for Micro-ROS
#include <stdio.h>                       // Standard I/O for debugging
#include <micro_ros_utilities/string_utilities.h>  // Utilities for handling strings


// ======== Micro-ROS Entity Declarations ========
rclc_support_t support;       // Micro-ROS execution context
rclc_executor_t executor;     // Manages execution of tasks (timers, subscribers)
rcl_allocator_t allocator;    // Handles memory allocation

rcl_node_t node;              // Defines the ROS 2 node

rcl_subscription_t pwm_suscriber;   // Subscribes to PWM control

std_msgs__msg__Int32 pwm_msg;       // Holds the pwm value (signed: sign = direction)

// ======== Macro Definitions ========
// Error handling macros
//Executes fn and returns false if it fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Executes a statement (X) every N milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// ======== Hardware Pin Definitions ========
#define IN1_PINA 18   //In1_a motor bridge h
#define IN1_PINB 19   //In1_b motor bridge h
#define PWM_PIN 5     // PWM connected to GPIO 5 (PWM output) 
#define PWM_RES 16      // PWM resolution (bits)
#define PWM_FRQ 980     // PWM Frequency  
#define PWM_CHNL 0     // PWM channel
#define MSG_MAX_VAL 65535
float pwm_set_point = 0.0;  // Stores the PWM magnitude value

// ======== Micro-ROS Connection State Machine ========
enum states {
  WAITING_AGENT,        // Waiting for ROS 2 agent connection
  AGENT_AVAILABLE,      // Agent detected
  AGENT_CONNECTED,      // Successfully connected
  AGENT_DISCONNECTED    // Connection lost
} state;


// ======== Function Prototypes ========
bool create_entities();
void destroy_entities();




// ======== Subscriber Callback: Adjusts Motor Speed and Direction ========
// Positive values  → forward  (PINA=HIGH, PINB=LOW)
// Negative values  → reverse  (PINA=LOW,  PINB=HIGH)
// PWM magnitude is abs(data) clamped to [0, MSG_MAX_VAL]
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*)msgin;
  int32_t val = msg->data;

  if (val >= 0) {
    digitalWrite(IN1_PINA, HIGH);
    digitalWrite(IN1_PINB, LOW);
  } else {
    digitalWrite(IN1_PINA, LOW);
    digitalWrite(IN1_PINB, HIGH);
  }

  pwm_set_point = constrain(abs(val), 0, MSG_MAX_VAL);
  ledcWrite(PWM_CHNL, (uint16_t)pwm_set_point);
}

// ======== Setup Function ========
void setup() {
  set_microros_transports();  // Initialize Micro-ROS communication
  
  // Setup PWM
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PINA,OUTPUT);
  pinMode(IN1_PINB, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHNL);       
}

// ======== Main Loop Function ========
void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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

// ======== ROS 2 Entity Creation and Cleanup Functions ========
bool create_entities()
{
  // Initialize Micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_pwm_controller", "", &support));

  // Initialize pwm Subscriber
  RCCHECK(rclc_subscription_init_default(
      &pwm_suscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "cmd_pwm"));


  // Initialize Executor
  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_suscriber, &pwm_msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&pwm_suscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
