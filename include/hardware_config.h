#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>
#include <stdint.h>

struct PlatformGeometry;

struct SimplePose6D {
    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;
};

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / PI)
#endif

const uint8_t ACTUATOR_PWM_PINS[6] = {
    2,
    3,
    4,
    5,
    6,
    7
};

const uint8_t ACTUATOR_DIR_PINS[6] = {
    8,
    9,
    10,
    11,
    12,
    13
};

const uint8_t ACTUATOR_ENABLE_PIN = 28;

const uint8_t LIMIT_SWITCH_MIN_PINS[6] = {
    15,
    17,
    19,
    21,
    23,
    25
};

const uint8_t LIMIT_SWITCH_MAX_PINS[6] = {
    16,
    18,
    20,
    22,
    24,
    26
};

const uint8_t CURRENT_SENSE_PINS[6] = {
    14,
    15,
    16,
    17,
    18,
    19
};

const uint8_t STATUS_LED_PIN = LED_BUILTIN;

const uint8_t DEBUG_SERIAL_TX_PIN = 1;
const uint8_t DEBUG_SERIAL_RX_PIN = 0;

const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 12;

enum MotorType {
    MOTOR_PWM_LINEAR = 0,
    MOTOR_SERVO_ROTARY = 1,
    MOTOR_STEPPER = 2
};

const int MOTOR_TYPE_CONFIG = MOTOR_PWM_LINEAR;

struct MotorConfig {
    double max_current;
    double max_velocity;
    double max_acceleration;
    double stall_current;
    double nominal_voltage;
    double resistance;
    double inductance;
    double back_emf_constant;
    double torque_constant;
};

const MotorConfig MOTOR_CONFIGS[6] = {
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05},
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05},
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05},
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05},
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05},
    {5.0, 100.0, 500.0, 2.5, 12.0, 2.4, 0.001, 0.05, 0.05}
};

struct CurrentSensorConfig {
    double sensitivity;
    double offset_voltage;
    double max_sense_current;
    double adc_reference;
};

const CurrentSensorConfig CURRENT_SENSOR_CONFIG = {
    0.066,
    1.65,
    5.0,
    3.3
};

const double DEFAULT_BASE_RADIUS = 150.0;
const double DEFAULT_BASE_ANGLE_OFFSET = 0.0;

const double DEFAULT_TOP_RADIUS = 100.0;
const double DEFAULT_TOP_ANGLE_OFFSET = PI / 6.0;

const double DEFAULT_MIN_LEG_LENGTH = 80.0;
const double DEFAULT_MAX_LEG_LENGTH = 200.0;
const double DEFAULT_ACTUATOR_OFFSET = 20.0;

const double DEFAULT_MAX_TRANSLATION = 50.0;
const double DEFAULT_MAX_ROTATION = PI / 6.0;

const int CONTROL_LOOP_FREQUENCY = 1000;
const int SAFETY_LOOP_FREQUENCY = 100;
const int TELEMETRY_LOOP_FREQUENCY = 100;

struct PIDParams {
    double kp;
    double ki;
    double kd;
    double integral_limit;
    double output_limit;
    double derivative_filter;
};

const PIDParams POSITION_PID_PARAMS = {
    .kp = 2.0,
    .ki = 0.0,
    .kd = 0.1,
    .integral_limit = 10.0,
    .output_limit = 1.0,
    .derivative_filter = 0.1
};

const PIDParams VELOCITY_PID_PARAMS = {
    .kp = 0.5,
    .ki = 0.1,
    .kd = 0.0,
    .integral_limit = 5.0,
    .output_limit = 1.0,
    .derivative_filter = 0.2
};

const double DEFAULT_MAX_VELOCITY = 100.0;
const double DEFAULT_MAX_ACCELERATION = 500.0;
const double DEFAULT_TRAJECTORY_DURATION = 1.0;

const double POSITION_TOLERANCE = 0.1;
const double ORIENTATION_TOLERANCE = 0.01;
const double SYNCHRONIZATION_TOLERANCE = 1.0;

const double OVERCURRENT_THRESHOLD = 5.5;
const double OVERCURRENT_DURATION = 100;

const double OVERTEMPERATURE_THRESHOLD = 80.0;

const double WORKSPACE_SAFETY_MARGIN = 5.0;
const double SINGULARITY_THRESHOLD = 0.95;

const int WATCHDOG_TIMEOUT_MS = 1000;

struct ActuatorCalibration {
    double length_offset;
    double length_scale;
    double velocity_offset;
    double velocity_scale;
    double current_offset;
    double current_scale;
    double min_valid_length;
    double max_valid_length;
    bool direction_inverted;
};

const ActuatorCalibration ACTUATOR_CALIBRATION[6] = {
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false}
};

struct LimitSwitchCalibration {
    bool min_switch_active_low;
    bool max_switch_active_low;
    double min_switch_position;
    double max_switch_position;
};

const LimitSwitchCalibration LIMIT_SWITCH_CALIBRATION[6] = {
    {true, true, 80.0, 200.0},
    {true, true, 80.0, 200.0},
    {true, true, 80.0, 200.0},
    {true, true, 80.0, 200.0},
    {true, true, 80.0, 200.0},
    {true, true, 80.0, 200.0}
};

const SimplePose6D HOME_POSITION = {
    .x = 0.0,
    .y = 0.0,
    .z = 150.0,
    .phi = 0.0,
    .theta = 0.0,
    .psi = 0.0
};

const int USB_CDC_BAUD_RATE = 12000000;
const int USB_CDC_TIMEOUT_MS = 100;

const int PROTOCOL_VERSION = 1;
const int MAX_PACKET_SIZE = 256;
const int PACKET_TIMEOUT_MS = 100;

const int TELEMETRY_QUEUE_SIZE = 20;
const int COMMAND_QUEUE_SIZE = 10;

#define DEBUG_LEVEL_NONE     0
#define DEBUG_LEVEL_ERROR    1
#define DEBUG_LEVEL_WARNING  2
#define DEBUG_LEVEL_INFO     3
#define DEBUG_LEVEL_DEBUG    4
#define DEBUG_LEVEL_VERBOSE  5

#define CURRENT_DEBUG_LEVEL DEBUG_LEVEL_INFO

const int DEBUG_SERIAL_BAUD_RATE = 115200;
const bool DEBUG_SERIAL_ENABLED = true;

const bool ENABLE_PERFORMANCE_MONITORING = true;
const int PERFORMANCE_MONITOR_INTERVAL_MS = 1000;

#define TASK_CONTROL_PRIORITY        (configMAX_PRIORITIES - 2)
#define TASK_SAFETY_PRIORITY         (configMAX_PRIORITIES - 3)
#define TASK_TELEMETRY_PRIORITY      (configMAX_PRIORITIES - 5)

#define TASK_CONTROL_STACK_SIZE      512
#define TASK_SAFETY_STACK_SIZE       256
#define TASK_TELEMETRY_STACK_SIZE    512

#define TOTAL_HEAP_SIZE              (128 * 1024)
#define STACK_WATERMARK_ENABLED      true

#define ENABLE_SERVO_MODE            false
#define ENABLE_STEPPER_MODE          false
#define ENABLE_LINEAR_MODE           true
#define ENABLE_ENCODER_FEEDBACK      false
#define ENABLE_CURRENT_SENSING       true
#define ENABLE_LIMIT_SWITCHES        true
#define ENABLE_TEMPERATURE_SENSING   false

#define ENABLE_FPU_OPTIMIZATION      true
#define ENABLE_DTCM_PLACEMENT        true
#define ENABLE_INLINE_FUNCTIONS      true

#if CURRENT_DEBUG_LEVEL >= DEBUG_LEVEL_DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#endif
