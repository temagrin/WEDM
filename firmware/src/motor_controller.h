// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <initializer_list>
#include "motor.h"


struct SpeedMotorState {
    int32_t speed;
    absolute_time_t lastStepTime;  
};

class StepperMotorController {
public:
    static constexpr int MAX_MOTORS = 8;
    StepperMotorController(StepperMotor* motorX, StepperMotor* motorY, std::initializer_list<StepperMotor*> motors);
    void initMotors();
    void handleMotors();
    void setMotorSpeed(int index, uint32_t speed);

private:
    StepperMotor* motorX = {nullptr};
    StepperMotor* motorY = {nullptr};
    StepperMotor* motors[MAX_MOTORS] = {nullptr};
    SpeedMotorState speedMotorStates[MAX_MOTORS];

    int motor_count = 0;
    absolute_time_t last_handle_time = get_absolute_time();
    void finalizeStep(StepperMotor* motor, absolute_time_t now);
    void finalizeMotorsSteps(absolute_time_t now);
    void stepsMotorSpeeds(absolute_time_t now);
    void stepsMotorPosition(absolute_time_t now);
};

#endif