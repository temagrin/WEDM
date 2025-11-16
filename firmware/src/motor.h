// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "motor_drivers.h"

class StepperMotor {
public:
    struct StepperConfig {
        uint8_t step_pin;
        uint8_t dir_pin;
        uint8_t en_pin;
        bool invert_step_pin;
        bool invert_dir_pin;
        bool invert_en_pin;
        uint8_t step_pulse_duration;
    };

    explicit StepperMotor(
        uint8_t step_pin,
        uint8_t dir_pin,
        uint8_t en_pin,
        const MotorDriver& driver_settings
        );

    void init();
    void doStep(bool dir, absolute_time_t now = get_absolute_time());
    void afterStep();
    void setPower(bool value);
    absolute_time_t getStepOnsetTime() const;
    uint8_t getStepPulseDuration() const;
    
private:
    StepperConfig config;
    absolute_time_t step_onset_time;
};

#endif