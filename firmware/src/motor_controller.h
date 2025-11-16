// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "pico/stdlib.h"
#include <initializer_list>
#include "motor.h"


class StepperMotorController {
public:
    static constexpr int MAX_MOTORS = 8;
    StepperMotorController(StepperMotor* motorX, StepperMotor* motorY, std::initializer_list<StepperMotor*> motors);
    void initMotors();
    void handleMotors();

private:
    StepperMotor* motorX = {nullptr};
    StepperMotor* motorY = {nullptr};
    StepperMotor* motors[MAX_MOTORS] = {nullptr};
    int motor_count = 0;
    absolute_time_t last_handle_time = get_absolute_time();
    void finalizeStep(StepperMotor* motor, absolute_time_t now);
    void finalizeMotors(absolute_time_t now);
};

#endif