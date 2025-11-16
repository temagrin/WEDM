// motor_controller.cpp
#include <fastmath.h>
#include "motor_controller.h"



StepperMotorController::StepperMotorController(StepperMotor* _motorX, StepperMotor* _motorY, std::initializer_list<StepperMotor*> motorList)
    : motorX(_motorX), motorY(_motorY), motor_count(0)
{
    int i = 0;
    for (auto m : motorList) {
        if (i >= MAX_MOTORS) break;
        motors[i++] = m;
    }
    motor_count = i;
}

void StepperMotorController::initMotors(){
    motorX->init();
    motorY->init();
    for (uint8_t i=0; i<motor_count; i++) {
        motors[i]->init();
    }
}

void StepperMotorController::finalizeStep(StepperMotor* motor, absolute_time_t now) {
    absolute_time_t step_onset_time = motor->getStepOnsetTime();
    uint8_t step_pulse_duration = motor->getStepPulseDuration();
    if (step_pulse_duration != 0) {
        absolute_time_t pulse_end_time = delayed_by_us(step_onset_time, step_pulse_duration);
        if (absolute_time_diff_us(pulse_end_time, now) <= 0) {
            motor->afterStep();
        }
    }
}

void StepperMotorController::finalizeMotorsSteps(absolute_time_t now) {    
    finalizeStep(motorX, now);
    finalizeStep(motorY, now);
    for (uint8_t i=0; i<motor_count; i++) {
        finalizeStep(motors[i], now);
    }    
}


void StepperMotorController::setMotorSpeed(int index, uint32_t speed) {
    if (index < 0 || index >= motor_count) return;
    speedMotorStates[index].speed = speed;
}


void StepperMotorController::stepsMotorSpeeds(absolute_time_t now) {
    for (int i = 0; i < motor_count; i++) {
        int32_t speed = speedMotorStates[i].speed;
        if (speed != 0) {
            int64_t step_interval_us = static_cast<int>(1e6 / abs(speed));
            int64_t dt = absolute_time_diff_us(speedMotorStates[i].lastStepTime, now);
            if (dt >= step_interval_us) {
                motors[i]->doStep(speed > 0, now);
                speedMotorStates[i].lastStepTime = now;
            }
        }
    }
}

void StepperMotorController::stepsMotorPosition(absolute_time_t now) {
    int64_t dt  = absolute_time_diff_us(last_handle_time, now);
    last_handle_time = now;
}


void StepperMotorController::handleMotors(){
    absolute_time_t now = get_absolute_time();
    stepsMotorPosition(now);
    stepsMotorSpeeds(now);
    finalizeMotorsSteps(now);
}