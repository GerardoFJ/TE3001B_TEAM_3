#pragma once

// ── WiFi & micro-ROS agent ────────────────────────────────────────────────────
#define WIFI_SSID        "Team4"
#define WIFI_PASS        "TeamFokin4"
#define AGENT_IP         192,168,1,53     // IP of machine running micro_ros_agent
#define AGENT_PORT       8889

// ── ROS ───────────────────────────────────────────────────────────────────────
#define PUBLISH_TOPIC    "/teleop/slave_force_direct"
#define FRAME_ID         "link_base"
#define PUBLISH_HZ       50

// ── GPIO pins ─────────────────────────────────────────────────────────────────
#define PIN_TRIGGER      4
#define PIN_ECHO         5
#define PIN_JOY_X        34
#define PIN_JOY_Y        35
#define PIN_JOY_SW       2
#define PIN_LED          13

// ── Spring / sensor parameters ────────────────────────────────────────────────
#define SPRING_K         1500.0f   // N/m  — spring constant
#define REST_DIST_CM     10.0f     // cm   — distance at rest (zero force); tune with D<cm> command
#define JOY_MAX_FORCE_N  6.0f      // N    — force at full joystick deflection
#define CAL_SAMPLES      50

// ── Deadbands (zero out noise below these thresholds) ─────────────────────────
#define JOY_DEADBAND_N   0.15f     // N    — joystick noise floor
#define FORCE_Z_MIN_N    0.3f      // N    — minimum spring force to report
#define DIST_DEADBAND_CM 0.8f      // cm   — ± around rest distance treated as zero
