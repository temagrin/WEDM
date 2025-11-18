// motor_controller.cpp
#include "motor_controller.h"


StepperMotorController::StepperMotorController(std::initializer_list<StepperMotor*> motorList): motorCount(0)
{
    int i = 0;
    for (auto m : motorList) {
        if (i >= MAX_MOTORS) break;
        motors[i++] = m;
    }
    motorCount = i;
}

void StepperMotorController::initMotors(){
    for (uint8_t i=0; i<motorCount; i++) {
        motors[i]->init();
    }
}


void StepperMotorController::finalizeStep(StepperMotor* motor){
    absolute_time_t now = get_absolute_time();
    const MotorState& state = motor->getMotorState();
    uint8_t stepPulseDuration = motor->getStepPulseDuration();
    if (stepPulseDuration == 0) { motor->stop(); return; }
    absolute_time_t elastedTime = delayed_by_us(state.stepOnsetTime, stepPulseDuration);
    if (absolute_time_diff_us(elastedTime, now) <= 0) return; 
    motor->afterStep();
}


void StepperMotorController::doStep(StepperMotor* motor){
    absolute_time_t now = get_absolute_time();
    const MotorState& state = motor->getMotorState();
    if (!state.running) return;    
    if (state.stepInterval == 0) { motor->stop(); return; }
    absolute_time_t elastedTime = delayed_by_us(state.stepLastTime, state.stepInterval);
    if (absolute_time_diff_us(elastedTime, now) <= 0) return;  
    if (state.calcSteps) {
        if (state.stepsRemaining<=0) return;
        motor->considStep();
    }
    motor->doStep(now);
}


void StepperMotorController::handleMotors(){
    for (uint8_t i=0; i<motorCount; i++) {
        doStep(motors[i]);
        finalizeStep(motors[i]);
    }
}