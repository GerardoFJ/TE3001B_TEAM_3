#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/wrench_stamped.h>

#include "config.h"

// ── micro-ROS handles ─────────────────────────────────────────────────────────
static rcl_publisher_t       publisher;
static geometry_msgs__msg__WrenchStamped msg;
static rclc_support_t        support;
static rcl_allocator_t       allocator;
static rcl_node_t            node;
static rcl_timer_t           timer;
static rclc_executor_t       executor;

// ── Sensor state ──────────────────────────────────────────────────────────────
static float springK    = SPRING_K;
static float restDistCm = REST_DIST_CM;
static int   centerX    = 2048;
static int   centerY    = 2048;

static float forceX = 0.0f;
static float forceY = 0.0f;
static float forceZ = 0.0f;

// ── Agent lifecycle ───────────────────────────────────────────────────────────
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static AgentState agentState = WAITING_AGENT;

// ── Sensor helpers ────────────────────────────────────────────────────────────
static void calibrateJoystick() {
    long sx = 0, sy = 0;
    Serial.println("Calibrating joystick... (keep centered)");
    for (int i = 0; i < CAL_SAMPLES; i++) {
        sx += analogRead(PIN_JOY_X);
        sy += analogRead(PIN_JOY_Y);
        delay(10);
    }
    centerX = (int)(sx / CAL_SAMPLES);
    centerY = (int)(sy / CAL_SAMPLES);
    Serial.printf("Center calibrated  X:%d  Y:%d\n", centerX, centerY);
}

static float measureDistanceCm() {
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIGGER, LOW);
    long dur = pulseIn(PIN_ECHO, HIGH, 30000);
    if (dur == 0) return -1.0f;
    return dur * 0.0343f / 2.0f;
}

static void readSensors() {
    // Z: spring compression measured by ultrasonic
    float dist = measureDistanceCm();
    if (dist > 0.0f && dist < restDistCm) {
        float compressionM = (restDistCm - dist) / 100.0f;
        forceZ = springK * compressionM;
    } else {
        forceZ = 0.0f;
    }

    // X/Y: joystick mapped to ±JOY_MAX_FORCE_N with deadband
    float nx = constrain((analogRead(PIN_JOY_X) - centerX) / 2048.0f, -1.0f, 1.0f);
    float ny = constrain((analogRead(PIN_JOY_Y) - centerY) / 2048.0f, -1.0f, 1.0f);
    float fx = nx * JOY_MAX_FORCE_N;
    float fy = ny * JOY_MAX_FORCE_N;
    forceX = (fabsf(fx) < JOY_DEADBAND_N) ? 0.0f : fx;
    forceY = (fabsf(fy) < JOY_DEADBAND_N) ? 0.0f : fy;

    // Z: apply minimum threshold to suppress noise
    if (forceZ < FORCE_Z_MIN_N) forceZ = 0.0f;
}

// ── Timer callback — runs at PUBLISH_HZ ──────────────────────────────────────
static void timerCallback(rcl_timer_t* /*t*/, int64_t /*last*/) {
    readSensors();

    int64_t ns = rmw_uros_epoch_nanos();
    msg.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
    msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
    msg.wrench.force.x = forceX;
    msg.wrench.force.y = forceY;
    msg.wrench.force.z = forceZ;

    rcl_publish(&publisher, &msg, NULL);

    bool active = (forceZ > 0.1f || fabsf(forceX) > 0.1f || fabsf(forceY) > 0.1f);
    digitalWrite(PIN_LED, active ? HIGH : LOW);
}

// ── micro-ROS entity management ───────────────────────────────────────────────
#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) return false; }

static bool createEntities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "force_sensor_hw", "", &support));
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, WrenchStamped),
        PUBLISH_TOPIC));
    RCCHECK(rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(1000 / PUBLISH_HZ),
        timerCallback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    rmw_uros_sync_session(1000);

    // frame_id points to static string — safe across callbacks
    msg.header.frame_id.data     = (char*)FRAME_ID;
    msg.header.frame_id.size     = strlen(FRAME_ID);
    msg.header.frame_id.capacity = strlen(FRAME_ID) + 1;

    Serial.println("micro-ROS entities created, publishing...");
    return true;
}

static void destroyEntities() {
    rmw_context_t* ctx = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0);
    rcl_publisher_fini(&publisher, &node);
    rcl_timer_fini(&timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_TRIGGER, OUTPUT);
    digitalWrite(PIN_TRIGGER, LOW);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_JOY_SW, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    calibrateJoystick();

    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.printf("\nWiFi OK  IP:%s\n", WiFi.localIP().toString().c_str());

    set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, IPAddress(AGENT_IP), AGENT_PORT);

    Serial.printf("k=%.0f N/m  rest=%.1f cm  topic=%s\n",
                  springK, restDistCm, PUBLISH_TOPIC);
    Serial.println("Cmds: T<N/m>  D<cm>  C(alibrate)");
}

void loop() {
    // Serial commands for runtime tuning
    if (Serial.available()) {
        char cmd = Serial.read();
        float val = Serial.parseFloat();
        while (Serial.available()) Serial.read();
        switch (cmd) {
            case 'T': case 't':
                if (val > 0) { springK = val; Serial.printf("k=%.0f N/m\n", springK); }
                break;
            case 'D': case 'd':
                if (val > 0) { restDistCm = val; Serial.printf("rest=%.1f cm\n", restDistCm); }
                break;
            case 'C': case 'c':
                calibrateJoystick();
                break;
        }
    }

    // micro-ROS reconnection state machine
    switch (agentState) {
        case WAITING_AGENT:
            // 500 ms timeout, 3 attempts — tolerates WiFi jitter
            if (rmw_uros_ping_agent(500, 3) == RMW_RET_OK)
                agentState = AGENT_AVAILABLE;
            else
                delay(500);   // don't hammer the network while waiting
            break;

        case AGENT_AVAILABLE:
            agentState = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (agentState == WAITING_AGENT) destroyEntities();
            break;

        case AGENT_CONNECTED:
            // Only ping every ~2 s (every 100 spin cycles × ~20 ms each)
            // A single missed ping no longer causes a disconnect
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            if (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
                agentState = AGENT_DISCONNECTED;
            }
            break;

        case AGENT_DISCONNECTED:
            destroyEntities();
            agentState = WAITING_AGENT;
            Serial.println("Agent lost, reconnecting...");
            break;
    }
}
