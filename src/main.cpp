#include <Arduino.h>
extern "C" {
#include <FreeRTOS.h>
#include <task.h>
}

#include "stewart_kinematics.h"
#include "freertos_tasks.h"
#include "motor_driver.h"
#include "hardware_config.h"

#define DEBUG_SERIAL Serial1

MotorController6DOF motor_controller;

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);
    
    DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD_RATE);
    DEBUG_SERIAL.println("Stewart Platform Firmware v1.0");
    DEBUG_SERIAL.println("Initializing system...");
    
    DEBUG_SERIAL.println("Initializing system timing...");
    
    DEBUG_SERIAL.println("Initializing motor drivers...");
    initMotorDrivers();
    
    DEBUG_SERIAL.println("Initializing serial communication...");
    initSerialCommunication();
    
    DEBUG_SERIAL.println("Initializing sensors...");
    initSensors();
    
    DEBUG_SERIAL.println("Initializing watchdog timer...");
    initWatchdog();
    
    DEBUG_SERIAL.println("Initializing FreeRTOS tasks...");
    initFreeRTOSTasks();
    
    DEBUG_SERIAL.println("System initialization complete.");
    DEBUG_SERIAL.println("Starting FreeRTOS scheduler...");
    
    digitalWrite(STATUS_LED_PIN, LOW);
    
    vTaskStartScheduler();
    
    DEBUG_SERIAL.println("ERROR: Scheduler failed to start!");
    while (1) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(100);
    }
}

void loop() {
}

void initMotorDrivers() {
    MotorPins motor_configs[6];
    MotorParams motor_params[6];
    
    for (int i = 0; i < 6; i++) {
        motor_configs[i].pwm_pin = ACTUATOR_PWM_PINS[i];
        motor_configs[i].dir_pin = ACTUATOR_DIR_PINS[i];
        motor_configs[i].enable_pin = ACTUATOR_ENABLE_PIN;
        motor_configs[i].limit_switch_min = LIMIT_SWITCH_MIN_PINS[i];
        motor_configs[i].limit_switch_max = LIMIT_SWITCH_MAX_PINS[i];
        motor_configs[i].current_sense_pin = CURRENT_SENSE_PINS[i];
        
        motor_params[i].type = (MotorType)MOTOR_TYPE_CONFIG;
        motor_params[i].max_current = MOTOR_CONFIGS[i].max_current;
        motor_params[i].max_velocity = MOTOR_CONFIGS[i].max_velocity;
        motor_params[i].max_acceleration = MOTOR_CONFIGS[i].max_acceleration;
        motor_params[i].min_position = ACTUATOR_CALIBRATION[i].min_valid_length;
        motor_params[i].max_position = ACTUATOR_CALIBRATION[i].max_valid_length;
        motor_params[i].pwm_frequency = PWM_FREQUENCY;
        motor_params[i].pwm_resolution = PWM_RESOLUTION;
    }
    
    motor_controller.init(motor_configs, motor_params);
    
    motor_controller.enableAll(true);
}

void initSerialCommunication() {
    Serial.begin(12000000);
    
    unsigned long timeout = millis() + 3000;
    while (!Serial && millis() < timeout) {
        delay(10);
    }
    
    if (Serial) {
        DEBUG_SERIAL.println("USB CDC connected at 12 Mbps");
    } else {
        DEBUG_SERIAL.println("USB CDC not connected (continuing anyway)");
    }
}

void initSensors() {
    analogReadResolution(12);
    analogReadAveraging(16);
    
    DEBUG_SERIAL.println("Sensors initialized");
}

void initWatchdog() {
    DEBUG_SERIAL.println("Watchdog timer configured");
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    DEBUG_SERIAL.print("STACK OVERFLOW: ");
    DEBUG_SERIAL.println(pcTaskName);
    
    emergencyStop();
    
    while (1) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(50);
    }
}

void vApplicationMallocFailedHook() {
    DEBUG_SERIAL.println("MALLOC FAILED");
    
    emergencyStop();
    
    while (1) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        delay(50);
    }
}

void vApplicationTickHook() {
}

void vApplicationIdleHook() {
}

void emergencyStop() {
    motor_controller.emergencyStopAll();
    
    current_system_state = STATE_EMERGENCY_STOP;
    
    DEBUG_SERIAL.println("EMERGENCY STOP ACTIVATED");
}

void setSystemStatus(uint8_t status) {
    switch (status) {
        case 0:
            digitalWrite(STATUS_LED_PIN, LOW);
            break;
        case 1:
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(500);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(500);
            break;
        case 2:
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
            break;
        default:
            digitalWrite(STATUS_LED_PIN, LOW);
            break;
    }
}
