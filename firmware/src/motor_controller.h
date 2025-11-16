// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <SerialUSB.h>
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
    void setMainAcceleration(int32_t mainAccelerationStepsPerSec2);
    void setPauseAcceleration(int32_t pauseAccelerationStepsPerSec2);
    
    void handleMotors();
    void handlePlaner();

    void setMotorSpeed(int index, uint32_t speed);
    void moveTo(int64_t newX, int64_t newY, uint32_t tool_start_speed, uint32_t tool_curise_speed, uint32_t tool_end_speed);
    void pause(uint8_t power);
    void resume();
    

private:
    StepperMotor* motorX = {nullptr};
    StepperMotor* motorY = {nullptr};
    StepperMotor* motors[MAX_MOTORS] = {nullptr};
    SpeedMotorState speedMotorStates[MAX_MOTORS];
    
    int32_t mainAccelerationStepsPerSec2 = 1000;
    int32_t pauseAccelerationStepsPerSec2 = 1000;
    
    int64_t currentX=0;
    int64_t currentY=0;
    int64_t currentX_printed=-1;
    int64_t currentY_printed=-1;
    
    
    StepperMotor* primaryMotor = {nullptr};
    StepperMotor* secondaryMotor = {nullptr};

    uint32_t primaryMotorCurrentStepIntervalUs = 0;
    uint32_t secondaryMotorCurrentStepIntervalUs = 0;
    
    bool mainAxisIsX=0;
    
    bool primaryMotorDirection;
    bool secondaryMotorDirection;

    int64_t primaryMotorStepsTotal;
    int64_t secondaryMotorStepsTotal;

    int64_t primaryMotorStepsDone;
    int64_t secondaryMotorStepsDone;

    int64_t primaryMotorStepsRemaining;
    int64_t secondaryMotorStepsRemaining;

    absolute_time_t primaryMotorStepsTimeOnset = get_absolute_time();
    absolute_time_t secondaryMotorStepsTimeOnset = get_absolute_time();
    
    int motor_count = 0;
    
    void finalizeStep(StepperMotor* motor, absolute_time_t now);
    void finalizeMotorsSteps(absolute_time_t now);
    void stepsMotorSpeeds(absolute_time_t now);
    void stepsMotorPosition(absolute_time_t now);
    void calculateProfile();
    void recalculateProfile();
};

#endif