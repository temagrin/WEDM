// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include <initializer_list>
#include "motor.h"
#include <fastmath.h>
#include <SerialUSB.h>


class StepperMotorController {
public:
    static constexpr int MAX_MOTORS = 8;
    StepperMotorController(std::initializer_list<StepperMotor*> motors);
    void initMotors();
    void handleMotors();
    void setInfininityRorationSpeed(StepperMotor* motor, int32_t speed);
    

private:

    StepperMotor* motors[MAX_MOTORS] = {nullptr};
    int motorCount = 0;
    void finalizeStep(StepperMotor* motor);
    void doStep(StepperMotor* motor);
};

#endif // MOTORCONTROLLER_H