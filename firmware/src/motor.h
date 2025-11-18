// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <SerialUSB.h>
#include "pico/stdlib.h"
#include "motor_drivers.h"

struct MotorState {
        absolute_time_t stepLastTime; // время начала шага, при завершение шага не изменяется
        absolute_time_t stepOnsetTime; // время начала шага, при завершение шага зануляется
        uint32_t stepInterval;
        bool direction;
        bool calcSteps;
        uint64_t stepsRemaining;
        uint64_t stepsDone;
        int64_t absoluteSteps;
        bool running; // шагаем ли мотор контроллером этим мотором
    };

struct StepperConfig {
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t en_pin;
    bool invert_step_pin;
    bool invert_dir_pin;
    bool invert_en_pin;
    uint8_t step_pulse_duration;
};

class StepperMotor {
public:

    explicit StepperMotor(
        uint8_t step_pin,
        uint8_t dir_pin,
        uint8_t en_pin,
        const MotorDriver& driver_settings
        );

    void init();
    void doStep(absolute_time_t now);
    void afterStep();
    void considStep();
    void setPower(bool power);
    void setInfininityRorationSpeed(int32_t speed);
    
    inline const uint8_t getStepPulseDuration() const { return config.step_pulse_duration; }
    inline const MotorState& getMotorState() const { return motorState; }
    
    inline void setStepInterval(double interval) { motorState.stepInterval = interval; }
    inline void setDirection(bool dir) { motorState.direction = dir; }
    inline void setCalcSteps(bool calc) { motorState.calcSteps = calc; }
    inline void setStepsRemaining(uint64_t steps) { motorState.stepsRemaining = steps; }
    inline void resetAbsoluteSteps() { motorState.absoluteSteps = 0; }
    inline void run() { motorState.running=true; }
    inline void stop() { motorState.running=false; }
    
private:
    StepperConfig config;
    MotorState motorState;
};

#endif // MOTOR_H