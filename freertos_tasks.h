#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include <Arduino.h>

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
}
#include "stewart_kinematics.h"

#define TASK_CONTROL_PRIORITY        (configMAX_PRIORITIES - 2)
#define TASK_SAFETY_PRIORITY         (configMAX_PRIORITIES - 3)
#define TASK_TELEMETRY_PRIORITY      (configMAX_PRIORITIES - 5)

#define TASK_CONTROL_STACK_SIZE      512
#define TASK_SAFETY_STACK_SIZE       256
#define TASK_TELEMETRY_STACK_SIZE    512

#define CONTROL_LOOP_FREQUENCY_HZ    1000
#define SAFETY_LOOP_FREQUENCY_HZ     100
#define TELEMETRY_LOOP_FREQUENCY_HZ  100

#define COMMAND_QUEUE_SIZE           10
#define TELEMETRY_QUEUE_SIZE         20

enum CommandType {
    CMD_MOVE_POSE = 0,
    CMD_MOVE_RELATIVE,
    CMD_HOME,
    CMD_EMERGENCY_STOP,
    CMD_SET_VELOCITY,
    CMD_SET_ACCELERATION,
    CMD_GET_TELEMETRY,
    CMD_CALIBRATE,
    CMD_RESET_ERROR
};

struct HostCommand {
    CommandType command_type;
    Pose6D target_pose;
    double velocity;
    double acceleration;
    double duration;
    uint32_t timestamp;
};

struct TelemetryData {
    Pose6D current_pose;
    ActuatorState actuators[6];
    double control_loop_period_us;
    double ik_computation_time_us;
    uint32_t loop_counter;
    uint8_t error_flags;
    uint8_t system_status;
    uint32_t timestamp;
};

enum SystemState {
    STATE_IDLE = 0,
    STATE_MOVING,
    STATE_HOMING,
    STATE_CALIBRATING,
    STATE_ERROR,
    STATE_EMERGENCY_STOP
};

struct PIDController {
    double kp;
    double ki;
    double kd;
    double integral;
    double previous_error;
    double output_min;
    double output_max;
};

struct ControlConfig {
    double max_velocity;
    double max_acceleration;
    double position_tolerance;
    double orientation_tolerance;
    PIDController position_pid;
    PIDController velocity_pid;
};

extern TaskHandle_t task_control_handle;
extern TaskHandle_t task_safety_handle;
extern TaskHandle_t task_telemetry_handle;

extern QueueHandle_t command_queue;
extern QueueHandle_t telemetry_queue;

extern SemaphoreHandle_t mutex_pose;
extern SemaphoreHandle_t mutex_actuators;

extern SystemState current_system_state;
extern ControlConfig control_config;

void taskControlLoop(void* pvParameters);
void taskSafetyMonitor(void* pvParameters);
void taskTelemetry(void* pvParameters);

void initFreeRTOSTasks();
void emergencyStop();
void clearErrorState();
void setControlConfig(const ControlConfig& config);
ControlConfig getControlConfig();
bool sendCommand(const HostCommand& command);
bool getTelemetry(TelemetryData& telemetry);

void initMotorDrivers();
void initSerialCommunication();
void initSensors();
void initWatchdog();

#endif
