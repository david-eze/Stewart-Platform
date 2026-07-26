#include "freertos_tasks.h"
#include <Arduino.h>
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
}
#include "stewart_kinematics.h"
#include "hardware_config.h"
#include "motor_driver.h"

TaskHandle_t task_control_handle = nullptr;
TaskHandle_t task_safety_handle = nullptr;
TaskHandle_t task_telemetry_handle = nullptr;

QueueHandle_t command_queue = nullptr;
QueueHandle_t telemetry_queue = nullptr;

SemaphoreHandle_t mutex_pose = nullptr;
SemaphoreHandle_t mutex_actuators = nullptr;

SystemState current_system_state = STATE_IDLE;
ControlConfig control_config;

StewartKinematics kinematics;
TrajectoryGenerator trajectory;

extern MotorController6DOF motor_controller;

Pose6D current_pose DTCM_ATTR;
ActuatorState current_actuators[6] DTCM_ATTR;

uint32_t control_loop_counter = 0;
double max_control_loop_period_us = 0.0;
double avg_control_loop_period_us = 0.0;

static StackType_t task_control_stack[TASK_CONTROL_STACK_SIZE] DTCM_ATTR;
static StackType_t task_safety_stack[TASK_SAFETY_STACK_SIZE] DTCM_ATTR;
static StackType_t task_telemetry_stack[TASK_TELEMETRY_STACK_SIZE] DTCM_ATTR;

static StaticTask_t task_control_buffer;
static StaticTask_t task_safety_buffer;
static StaticTask_t task_telemetry_buffer;

static uint8_t command_queue_storage_area[COMMAND_QUEUE_SIZE * sizeof(HostCommand)] DTCM_ATTR;
static StaticQueue_t command_queue_buffer;

static uint8_t telemetry_queue_storage_area[TELEMETRY_QUEUE_SIZE * sizeof(TelemetryData)] DTCM_ATTR;
static StaticQueue_t telemetry_queue_buffer;

static StaticSemaphore_t mutex_pose_buffer;
static StaticSemaphore_t mutex_actuators_buffer;

void taskControlLoop(void* pvParameters) {
    (void)pvParameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t control_period = pdMS_TO_TICKS(1);
    
    HostCommand current_command;
    bool command_available = false;
    
    for (int i = 0; i < 6; i++) {
        current_actuators[i].current_length = 150.0;
        current_actuators[i].target_length = 150.0;
        current_actuators[i].velocity = 0.0;
        current_actuators[i].acceleration = 0.0;
        current_actuators[i].error_state = false;
    }
    
    current_pose = {0.0, 0.0, 150.0, 0.0, 0.0, 0.0};
    
    trajectory.initTrajectory(current_pose, current_pose, 1.0);
    
    for (;;) {
        uint32_t loop_start_time = micros();
        
        if (current_system_state == STATE_EMERGENCY_STOP) {
            for (int i = 0; i < 6; i++) {
                analogWrite(ACTUATOR_PWM_PINS[i], 0);
            }
            digitalWrite(ACTUATOR_ENABLE_PIN, LOW);
            
            motor_controller.emergencyStopAll();
            
            vTaskDelayUntil(&last_wake_time, control_period);
            continue;
        }
        
        if (xQueueReceive(command_queue, &current_command, 0) == pdTRUE) {
            command_available = true;
            
            switch (current_command.command_type) {
                case CMD_MOVE_POSE:
                    current_system_state = STATE_MOVING;
                    trajectory.initTrajectory(current_pose, current_command.target_pose, 
                                            current_command.duration, true);
                    break;
                    
                case CMD_MOVE_RELATIVE: {
                    Pose6D relative_pose = current_pose;
                    relative_pose.x += current_command.target_pose.x;
                    relative_pose.y += current_command.target_pose.y;
                    relative_pose.z += current_command.target_pose.z;
                    relative_pose.phi += current_command.target_pose.phi;
                    relative_pose.theta += current_command.target_pose.theta;
                    relative_pose.psi += current_command.target_pose.psi;
                    
                    current_system_state = STATE_MOVING;
                    trajectory.initTrajectory(current_pose, relative_pose, 
                                            current_command.duration, true);
                    break;
                }
                
                case CMD_HOME:
                    current_system_state = STATE_HOMING;
                    trajectory.initTrajectory(current_pose, {0.0, 0.0, 150.0, 0.0, 0.0, 0.0}, 
                                            2.0, true);
                    break;
                    
                case CMD_EMERGENCY_STOP:
                    emergencyStop();
                    break;
                    
                case CMD_SET_VELOCITY:
                    control_config.max_velocity = current_command.velocity;
                    break;
                    
                case CMD_SET_ACCELERATION:
                    control_config.max_acceleration = current_command.acceleration;
                    break;
                    
                default:
                    break;
            }
        }
        
        if (current_system_state == STATE_MOVING || current_system_state == STATE_HOMING) {
            Pose6D next_pose = trajectory.getNextPose(0.001);
            
            if (trajectory.isTrajectoryComplete()) {
                current_system_state = STATE_IDLE;
            }
            
            if (xSemaphoreTake(mutex_pose, portMAX_DELAY) == pdTRUE) {
                current_pose = next_pose;
                xSemaphoreGive(mutex_pose);
            }
            
            IKResult ik_result = kinematics.solveIK(current_pose);
            
            if (ik_result.solution_valid) {
                for (int i = 0; i < 6; i++) {
                    double position_error = ik_result.actuators[i].target_length - 
                                           current_actuators[i].current_length;
                    
                    double pid_output = control_config.position_pid.kp * position_error;
                    double velocity_feedforward = 0.0;
                    double control_output = pid_output + velocity_feedforward;
                    
                    if (control_output > control_config.position_pid.output_max) {
                        control_output = control_config.position_pid.output_max;
                    } else if (control_output < control_config.position_pid.output_min) {
                        control_output = control_config.position_pid.output_min;
                    }
                    
                    int pwm_value = (int)(control_output * 2047.5 + 2047.5);
                    pwm_value = constrain(pwm_value, 0, 4095);
                    
                    analogWrite(ACTUATOR_PWM_PINS[i], pwm_value);
                    
                    if (control_output >= 0) {
                        digitalWrite(ACTUATOR_DIR_PINS[i], HIGH);
                    } else {
                        digitalWrite(ACTUATOR_DIR_PINS[i], LOW);
                    }
                    
                    current_actuators[i].target_length = ik_result.actuators[i].target_length;
                    current_actuators[i].current_length += control_output * 0.001;
                }
            } else {
                current_system_state = STATE_ERROR;
            }
        }
        
        control_loop_counter++;
        
        uint32_t loop_end_time = micros();
        double loop_period = loop_end_time - loop_start_time;
        
        if (loop_period > max_control_loop_period_us) {
            max_control_loop_period_us = loop_period;
        }
        
        avg_control_loop_period_us = 0.95 * avg_control_loop_period_us + 0.05 * loop_period;
        
        vTaskDelayUntil(&last_wake_time, control_period);
    }
}

void taskSafetyMonitor(void* pvParameters) {
    (void)pvParameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t safety_period = pdMS_TO_TICKS(10);
    
    for (;;) {
        bool limit_error = false;
        for (int i = 0; i < 6; i++) {
            if (current_actuators[i].current_length < kinematics.getGeometry().min_leg_length ||
                current_actuators[i].current_length > kinematics.getGeometry().max_leg_length) {
                limit_error = true;
                current_actuators[i].error_state = true;
            }
        }
        
        if (limit_error && current_system_state != STATE_ERROR) {
            current_system_state = STATE_ERROR;
        }
        
        vTaskDelayUntil(&last_wake_time, safety_period);
    }
}

void taskTelemetry(void* pvParameters) {
    (void)pvParameters;
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t telemetry_period = pdMS_TO_TICKS(10);
    
    TelemetryData telemetry;
    
    for (;;) {
        if (Serial.available() > 0) {
            String command_str = Serial.readStringUntil('\n');
            
            if (command_str.startsWith("MOVE")) {
                HostCommand cmd;
                cmd.command_type = CMD_MOVE_POSE;
                cmd.duration = 1.0;
                cmd.timestamp = millis();
                
                xQueueSend(command_queue, &cmd, 0);
            } else if (command_str.startsWith("HOME")) {
                HostCommand cmd;
                cmd.command_type = CMD_HOME;
                cmd.timestamp = millis();
                
                xQueueSend(command_queue, &cmd, 0);
            } else if (command_str.startsWith("STOP")) {
                HostCommand cmd;
                cmd.command_type = CMD_EMERGENCY_STOP;
                cmd.timestamp = millis();
                
                xQueueSend(command_queue, &cmd, 0);
            }
        }
        
        telemetry.loop_counter = control_loop_counter;
        telemetry.control_loop_period_us = avg_control_loop_period_us;
        telemetry.error_flags = (current_system_state == STATE_ERROR) ? 0x01 : 0x00;
        telemetry.system_status = (uint8_t)current_system_state;
        telemetry.timestamp = millis();
        
        if (xSemaphoreTake(mutex_pose, portMAX_DELAY) == pdTRUE) {
            telemetry.current_pose = current_pose;
            xSemaphoreGive(mutex_pose);
        }
        
        if (xSemaphoreTake(mutex_actuators, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < 6; i++) {
                telemetry.actuators[i] = current_actuators[i];
            }
            xSemaphoreGive(mutex_actuators);
        }
        
        Serial.print("TEL ");
        Serial.print(telemetry.current_pose.x, 3);
        Serial.print(" ");
        Serial.print(telemetry.current_pose.y, 3);
        Serial.print(" ");
        Serial.print(telemetry.current_pose.z, 3);
        Serial.print(" ");
        Serial.print(telemetry.current_pose.phi * 57.29578, 2);
        Serial.print(" ");
        Serial.print(telemetry.current_pose.theta * 57.29578, 2);
        Serial.print(" ");
        Serial.print(telemetry.current_pose.psi * 57.29578, 2);
        Serial.print(" ");
        Serial.print(telemetry.control_loop_period_us, 1);
        Serial.print(" ");
        Serial.println(telemetry.system_status);
        
        xQueueOverwrite(telemetry_queue, &telemetry);
        
        vTaskDelayUntil(&last_wake_time, telemetry_period);
    }
}

void initFreeRTOSTasks() {
    mutex_pose = xSemaphoreCreateMutexStatic(&mutex_pose_buffer);
    mutex_actuators = xSemaphoreCreateMutexStatic(&mutex_actuators_buffer);
    
    command_queue = xQueueCreateStatic(COMMAND_QUEUE_SIZE, sizeof(HostCommand),
                                       command_queue_storage_area, &command_queue_buffer);
    telemetry_queue = xQueueCreateStatic(TELEMETRY_QUEUE_SIZE, sizeof(TelemetryData),
                                         telemetry_queue_storage_area, &telemetry_queue_buffer);
    
    control_config.max_velocity = DEFAULT_MAX_VELOCITY;
    control_config.max_acceleration = DEFAULT_MAX_ACCELERATION;
    control_config.position_tolerance = POSITION_TOLERANCE;
    control_config.orientation_tolerance = ORIENTATION_TOLERANCE;
    
    control_config.position_pid.kp = POSITION_PID_PARAMS.kp;
    control_config.position_pid.ki = POSITION_PID_PARAMS.ki;
    control_config.position_pid.kd = POSITION_PID_PARAMS.kd;
    control_config.position_pid.integral = 0.0;
    control_config.position_pid.previous_error = 0.0;
    control_config.position_pid.output_min = -POSITION_PID_PARAMS.output_limit;
    control_config.position_pid.output_max = POSITION_PID_PARAMS.output_limit;
    
    control_config.velocity_pid.kp = VELOCITY_PID_PARAMS.kp;
    control_config.velocity_pid.ki = VELOCITY_PID_PARAMS.ki;
    control_config.velocity_pid.kd = VELOCITY_PID_PARAMS.kd;
    control_config.velocity_pid.integral = 0.0;
    control_config.velocity_pid.previous_error = 0.0;
    control_config.velocity_pid.output_min = -VELOCITY_PID_PARAMS.output_limit;
    control_config.velocity_pid.output_max = VELOCITY_PID_PARAMS.output_limit;
    
    task_control_handle = xTaskCreateStatic(
        taskControlLoop,
        "ControlLoop",
        TASK_CONTROL_STACK_SIZE,
        nullptr,
        TASK_CONTROL_PRIORITY,
        task_control_stack,
        &task_control_buffer
    );
    
    task_safety_handle = xTaskCreateStatic(
        taskSafetyMonitor,
        "SafetyMonitor",
        TASK_SAFETY_STACK_SIZE,
        nullptr,
        TASK_SAFETY_PRIORITY,
        task_safety_stack,
        &task_safety_buffer
    );
    
    task_telemetry_handle = xTaskCreateStatic(
        taskTelemetry,
        "Telemetry",
        TASK_TELEMETRY_STACK_SIZE,
        nullptr,
        TASK_TELEMETRY_PRIORITY,
        task_telemetry_stack,
        &task_telemetry_buffer
    );
}

void emergencyStop() {
    current_system_state = STATE_EMERGENCY_STOP;
    
    motor_controller.emergencyStopAll();
    
    for (int i = 0; i < 6; i++) {
        analogWrite(ACTUATOR_PWM_PINS[i], 0);
    }
    digitalWrite(ACTUATOR_ENABLE_PIN, LOW);
}

void clearErrorState() {
    if (current_system_state == STATE_ERROR || current_system_state == STATE_EMERGENCY_STOP) {
        current_system_state = STATE_IDLE;
        
        for (int i = 0; i < 6; i++) {
            current_actuators[i].error_state = false;
        }
        
        motor_controller.enableAll(true);
        
        digitalWrite(ACTUATOR_ENABLE_PIN, HIGH);
    }
}

void setControlConfig(const ControlConfig& config) {
    control_config = config;
}

ControlConfig getControlConfig() {
    return control_config;
}

bool sendCommand(const HostCommand& command) {
    return xQueueSend(command_queue, &command, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool getTelemetry(TelemetryData& telemetry) {
    return xQueueReceive(telemetry_queue, &telemetry, pdMS_TO_TICKS(10)) == pdTRUE;
}

void initMotorDrivers() {
    for (int i = 0; i < 6; i++) {
        pinMode(ACTUATOR_PWM_PINS[i], OUTPUT);
        pinMode(ACTUATOR_DIR_PINS[i], OUTPUT);
        analogWriteFrequency(ACTUATOR_PWM_PINS[i], PWM_FREQUENCY);
        analogWriteResolution(PWM_RESOLUTION);
        analogWrite(ACTUATOR_PWM_PINS[i], 0);
        digitalWrite(ACTUATOR_DIR_PINS[i], LOW);
    }
    
    pinMode(ACTUATOR_ENABLE_PIN, OUTPUT);
    digitalWrite(ACTUATOR_ENABLE_PIN, HIGH);
}

void initSerialCommunication() {
    Serial.begin(12000000);
    while (!Serial && millis() < 3000);
}

void initSensors() {
}

void initWatchdog() {
}
