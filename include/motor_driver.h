#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <stdint.h>

enum MotorType {
    MOTOR_PWM_LINEAR = 0,
    MOTOR_SERVO_ROTARY,
    MOTOR_STEPPER,
    MOTOR_DC_BRUSHED
};

struct MotorPins {
    uint8_t pwm_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t limit_switch_min;
    uint8_t limit_switch_max;
    uint8_t current_sense_pin;
};

struct MotorParams {
    MotorType type;
    double max_current;
    double max_velocity;
    double max_acceleration;
    double min_position;
    double max_position;
    double pwm_frequency;
    int pwm_resolution;
};

struct MotorState {
    double current_position;
    double target_position;
    double current_velocity;
    double current_current;
    double temperature;
    bool enabled;
    bool error_state;
    bool limit_min_triggered;
    bool limit_max_triggered;
};

class MotorDriver {
private:
    MotorPins pins;
    MotorParams params;
    MotorState state;
    
    double pid_integral;
    double pid_previous_error;
    uint32_t last_update_time;
    
    double readCurrent();
    void readLimitSwitches(bool& min_limit, bool& max_limit);
    void setPWM(double duty_cycle, bool direction);
    void setServoAngle(double angle);
    void stepStepper(int steps, bool direction);

public:
    MotorDriver();
    
    void init(const MotorPins& pin_config, const MotorParams& motor_params);
    void setTargetPosition(double position);
    void setTargetVelocity(double velocity);
    void enable(bool enable);
    void emergencyStop();
    void update(double dt);
    
    MotorState getState() const;
    double getPosition() const;
    bool atTarget(double tolerance = 0.1) const;
    bool hasError() const;
    void clearError();
    void home(double homing_velocity);
};

class MotorController6DOF {
private:
    MotorDriver motors[6];
    bool system_enabled;
    bool emergency_stop_active;
    double max_synchronization_error;
    
public:
    MotorController6DOF();
    
    void init(const MotorPins motor_configs[6], const MotorParams motor_params[6]);
    void setTargetPositions(const double positions[6]);
    void setTargetVelocities(const double velocities[6]);
    void enableAll(bool enable);
    void emergencyStopAll();
    void updateAll(double dt);
    
    void getAllStates(MotorState states[6]) const;
    void getAllPositions(double positions[6]) const;
    bool allAtTarget(double tolerance = 0.1) const;
    bool hasAnyError() const;
    void homeAll(double homing_velocity);
    void setSyncTolerance(double tolerance);
    bool isSynchronized() const;
};

void initPWMHardware(uint8_t pin, double frequency, int resolution);
void initServoHardware(uint8_t pin);
void initStepperHardware(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin);

#endif
