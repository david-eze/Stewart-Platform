#include "motor_driver.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

using std::fabs;
using std::abs;

MotorDriver::MotorDriver() {
    pid_integral = 0.0;
    pid_previous_error = 0.0;
    last_update_time = micros();
    
    state.current_position = 0.0;
    state.target_position = 0.0;
    state.current_velocity = 0.0;
    state.current_current = 0.0;
    state.temperature = 25.0;
    state.enabled = false;
    state.error_state = false;
    state.limit_min_triggered = false;
    state.limit_max_triggered = false;
}

void MotorDriver::init(const MotorPins& pin_config, const MotorParams& motor_params) {
    pins = pin_config;
    params = motor_params;
    
    pinMode(pins.pwm_pin, OUTPUT);
    pinMode(pins.dir_pin, OUTPUT);
    
    if (pins.enable_pin != 255) {
        pinMode(pins.enable_pin, OUTPUT);
        digitalWrite(pins.enable_pin, LOW);
    }
    
    if (pins.limit_switch_min != 255) {
        pinMode(pins.limit_switch_min, INPUT_PULLUP);
    }
    
    if (pins.limit_switch_max != 255) {
        pinMode(pins.limit_switch_max, INPUT_PULLUP);
    }
    
    if (pins.current_sense_pin != 255) {
        pinMode(pins.current_sense_pin, INPUT);
    }
    
    switch (params.type) {
        case MOTOR_PWM_LINEAR:
        case MOTOR_DC_BRUSHED:
            initPWMHardware(pins.pwm_pin, params.pwm_frequency, params.pwm_resolution);
            break;
            
        case MOTOR_SERVO_ROTARY:
            initServoHardware(pins.pwm_pin);
            break;
            
        case MOTOR_STEPPER:
            initStepperHardware(pins.pwm_pin, pins.dir_pin, pins.enable_pin);
            break;
    }
    
    setPWM(0.0, true);
}

double MotorDriver::readCurrent() {
    if (pins.current_sense_pin == 255) {
        return 0.0;
    }
    
    int adc_value = analogRead(pins.current_sense_pin);
    
    const double volts_per_amp = CURRENT_SENSOR_CONFIG.sensitivity;
    const double adc_to_volts = CURRENT_SENSOR_CONFIG.adc_reference / 4095.0;
    
    double voltage = adc_value * adc_to_volts;
    double current = (voltage - CURRENT_SENSOR_CONFIG.offset_voltage) / volts_per_amp;
    
    return current;
}

void MotorDriver::readLimitSwitches(bool& min_limit, bool& max_limit) {
    min_limit = false;
    max_limit = false;
    
    if (pins.limit_switch_min != 255) {
        min_limit = (digitalRead(pins.limit_switch_min) == LOW);
    }
    
    if (pins.limit_switch_max != 255) {
        max_limit = (digitalRead(pins.limit_switch_max) == LOW);
    }
}

void MotorDriver::setPWM(double duty_cycle, bool direction) {
    if (duty_cycle < 0.0) duty_cycle = 0.0;
    if (duty_cycle > 1.0) duty_cycle = 1.0;
    
    digitalWrite(pins.dir_pin, direction ? HIGH : LOW);
    
    int pwm_value = (int)(duty_cycle * (1 << params.pwm_resolution));
    pwm_value = constrain(pwm_value, 0, (1 << params.pwm_resolution) - 1);
    
    analogWrite(pins.pwm_pin, pwm_value);
}

void MotorDriver::setServoAngle(double angle) {
    const double min_pulse = 1000.0;
    const double max_pulse = 2000.0;
    const double min_angle = -90.0;
    const double max_angle = 90.0;
    
    angle = constrain(angle, min_angle, max_angle);
    
    double pulse_width = min_pulse + (angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle);
    
    const double servo_period = 20000.0;
    double duty_cycle = pulse_width / servo_period;
    
    setPWM(duty_cycle, true);
}

void MotorDriver::stepStepper(int steps, bool direction) {
    digitalWrite(pins.dir_pin, direction ? HIGH : LOW);
    
    for (int i = 0; i < (steps < 0 ? -steps : steps); i++) {
        digitalWrite(pins.pwm_pin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pins.pwm_pin, LOW);
        delayMicroseconds(50);
    }
}

void MotorDriver::setTargetPosition(double position) {
    position = constrain(position, params.min_position, params.max_position);
    state.target_position = position;
}

void MotorDriver::setTargetVelocity(double velocity) {
    velocity = constrain(velocity, -params.max_velocity, params.max_velocity);
    state.current_velocity = velocity;
}

void MotorDriver::enable(bool enable) {
    state.enabled = enable;
    
    if (pins.enable_pin != 255) {
        digitalWrite(pins.enable_pin, enable ? HIGH : LOW);
    }
    
    if (!enable) {
        setPWM(0.0, true);
    }
}

void MotorDriver::emergencyStop() {
    enable(false);
    state.current_velocity = 0.0;
    state.error_state = true;
}

void MotorDriver::update(double dt) {
    if (!state.enabled || state.error_state) {
        return;
    }
    
    uint32_t current_time = micros();
    double elapsed = (current_time - last_update_time) / 1e6;
    last_update_time = current_time;
    
    readLimitSwitches(state.limit_min_triggered, state.limit_max_triggered);
    
    if (state.limit_min_triggered && state.current_velocity < 0) {
        state.current_velocity = 0.0;
        state.current_position = params.min_position;
    }
    
    if (state.limit_max_triggered && state.current_velocity > 0) {
        state.current_velocity = 0.0;
        state.current_position = params.max_position;
    }
    
    double position_error = state.target_position - state.current_position;
    double kp = 10.0;
    
    double velocity_command = kp * position_error;
    
    velocity_command = constrain(velocity_command, -params.max_velocity, params.max_velocity);
    
    state.current_velocity = velocity_command;
    
    state.current_position += state.current_velocity * dt;
    
    state.current_position = constrain(state.current_position, params.min_position, params.max_position);
    
    state.current_current = readCurrent();
    
    if (state.current_current > params.max_current) {
        state.error_state = true;
        enable(false);
    }
    
    switch (params.type) {
        case MOTOR_PWM_LINEAR:
        case MOTOR_DC_BRUSHED: {
            double duty_cycle = state.current_velocity / params.max_velocity;
            bool direction = state.current_velocity >= 0;
            setPWM(fabs(duty_cycle), direction);
            break;
        }
        
        case MOTOR_SERVO_ROTARY:
            setServoAngle(state.current_position);
            break;
            
        case MOTOR_STEPPER: {
            const double steps_per_rev = 200.0;
            const double mm_per_rev = 1.0;
            
            double steps_per_sec = (state.current_velocity / mm_per_rev) * steps_per_rev;
            int steps = (int)(steps_per_sec * dt);
            bool direction = state.current_velocity >= 0;
            
            if (abs(steps) > 0) {
                stepStepper(steps, direction);
            }
            break;
        }
    }
}

MotorState MotorDriver::getState() const {
    return state;
}

double MotorDriver::getPosition() const {
    return state.current_position;
}

bool MotorDriver::atTarget(double tolerance) const {
    return fabs(state.target_position - state.current_position) < tolerance;
}

bool MotorDriver::hasError() const {
    return state.error_state;
}

void MotorDriver::clearError() {
    state.error_state = false;
    state.limit_min_triggered = false;
    state.limit_max_triggered = false;
}

void MotorDriver::home(double homing_velocity) {
    state.target_position = params.min_position;
    state.current_velocity = -homing_velocity;
    
    while (!state.limit_min_triggered && !state.error_state) {
        update(0.001);
        delay(1);
    }
    
    state.current_position = params.min_position;
    state.current_velocity = 0.0;
}

MotorController6DOF::MotorController6DOF() {
    system_enabled = false;
    emergency_stop_active = false;
    max_synchronization_error = 1.0;
}

void MotorController6DOF::init(const MotorPins motor_configs[6], const MotorParams motor_params[6]) {
    for (int i = 0; i < 6; i++) {
        motors[i].init(motor_configs[i], motor_params[i]);
    }
}

void MotorController6DOF::setTargetPositions(const double positions[6]) {
    for (int i = 0; i < 6; i++) {
        motors[i].setTargetPosition(positions[i]);
    }
}

void MotorController6DOF::setTargetVelocities(const double velocities[6]) {
    for (int i = 0; i < 6; i++) {
        motors[i].setTargetVelocity(velocities[i]);
    }
}

void MotorController6DOF::enableAll(bool enable) {
    system_enabled = enable;
    for (int i = 0; i < 6; i++) {
        motors[i].enable(enable);
    }
}

void MotorController6DOF::emergencyStopAll() {
    emergency_stop_active = true;
    for (int i = 0; i < 6; i++) {
        motors[i].emergencyStop();
    }
}

void MotorController6DOF::updateAll(double dt) {
    if (emergency_stop_active) {
        return;
    }
    
    for (int i = 0; i < 6; i++) {
        motors[i].update(dt);
    }
}

void MotorController6DOF::getAllStates(MotorState states[6]) const {
    for (int i = 0; i < 6; i++) {
        states[i] = motors[i].getState();
    }
}

void MotorController6DOF::getAllPositions(double positions[6]) const {
    for (int i = 0; i < 6; i++) {
        positions[i] = motors[i].getPosition();
    }
}

bool MotorController6DOF::allAtTarget(double tolerance) const {
    for (int i = 0; i < 6; i++) {
        if (!motors[i].atTarget(tolerance)) {
            return false;
        }
    }
    return true;
}

bool MotorController6DOF::hasAnyError() const {
    for (int i = 0; i < 6; i++) {
        if (motors[i].hasError()) {
            return true;
        }
    }
    return false;
}

void MotorController6DOF::homeAll(double homing_velocity) {
    for (int i = 0; i < 6; i++) {
        motors[i].home(homing_velocity);
    }
}

void MotorController6DOF::setSyncTolerance(double tolerance) {
    max_synchronization_error = tolerance;
}

bool MotorController6DOF::isSynchronized() const {
    double positions[6];
    getAllPositions(positions);
    
    double avg_position = 0.0;
    for (int i = 0; i < 6; i++) {
        avg_position += positions[i];
    }
    avg_position /= 6.0;
    
    for (int i = 0; i < 6; i++) {
        if (fabs(positions[i] - avg_position) > max_synchronization_error) {
            return false;
        }
    }
    
    return true;
}

void initPWMHardware(uint8_t pin, double frequency, int resolution) {
    analogWriteFrequency(pin, frequency);
    analogWriteResolution(resolution);
}

void initServoHardware(uint8_t pin) {
    analogWriteFrequency(pin, 50);
    analogWriteResolution(12);
}

void initStepperHardware(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin) {
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    
    if (enable_pin != 255) {
        pinMode(enable_pin, OUTPUT);
        digitalWrite(enable_pin, LOW);
    }
    
    digitalWrite(step_pin, LOW);
    digitalWrite(dir_pin, LOW);
}
