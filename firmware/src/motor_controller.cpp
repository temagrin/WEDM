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

void StepperMotorController::setMainAcceleration(int32_t _mainAccelerationStepsPerSec2){
    mainAccelerationStepsPerSec2 = _mainAccelerationStepsPerSec2;
}
void StepperMotorController::setPauseAcceleration(int32_t _pauseAccelerationStepsPerSec2){
    pauseAccelerationStepsPerSec2 = _pauseAccelerationStepsPerSec2;
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


void StepperMotorController::setMotorSpeed(int index, uint32_t speed) {
    if (index < 0 || index >= motor_count) return;
    speedMotorStates[index].speed = speed;
}


void StepperMotorController::stepsMotorPosition(absolute_time_t now) {
    int64_t primaryDT  = absolute_time_diff_us(primaryMotorStepsTimeOnset, now);
    if (primaryDT >= primaryMotorCurrentStepIntervalUs && primaryMotorCurrentStepIntervalUs !=0) {
            if (primaryMotorStepsRemaining>0){ // если есть шаги - шагаем
                primaryMotor->doStep(primaryMotorDirection, now);
                primaryMotorStepsRemaining--;
                primaryMotorStepsDone++;
                if (mainAxisIsX){
                    primaryMotorDirection ? currentX++:currentX-- ;
                } else {
                    primaryMotorDirection ? currentY++:currentY-- ;
                }
            } // шагов нет но интервал остался - все равно обновляем
            primaryMotorStepsTimeOnset = now;
        }
    int64_t secondaryDT  = absolute_time_diff_us(secondaryMotorStepsTimeOnset, now);
    if (secondaryDT >= secondaryMotorCurrentStepIntervalUs && secondaryMotorCurrentStepIntervalUs !=0) {
            if (secondaryMotorStepsRemaining>0){ // если есть шаги - шагаем
                secondaryMotor->doStep(secondaryMotorDirection, now);
                secondaryMotorStepsRemaining--;
                secondaryMotorStepsDone++;
                if (mainAxisIsX){
                    secondaryMotorDirection ? currentY++:currentY-- ;
                } else {
                    secondaryMotorDirection ? currentX++:currentX-- ;
                }
            } // шагов нет но интервал остался - все равно обновляем
            secondaryMotorStepsTimeOnset = now;
        }
}


void StepperMotorController::calculateProfile(){

}

void StepperMotorController::recalculateProfile(){
    
}

void StepperMotorController::pause(uint8_t power){

}

void StepperMotorController::resume(){

}
    

void StepperMotorController::moveTo(int64_t newX, int64_t newY, uint32_t tool_start_speed, uint32_t tool_curise_speed, uint32_t tool_end_speed) {
    int64_t dx = newX - currentX;
    int64_t dy = newY - currentY;
    bool dirX = dx>=0;
    bool dirY = dy>=0;

    if (abs(dx) >= abs(dy)) {
    primaryMotor = motorX; secondaryMotor = motorY;
    primaryMotorStepsTotal = abs(dx); secondaryMotorStepsTotal = abs(dy);
    primaryMotorDirection = dirX; secondaryMotorDirection = dirY;
    mainAxisIsX = true; // для расчета currentX currentY при выполнение шага, для обратного перевода с первичного мотора на осевой, не забыть мне!
    } else {
    primaryMotor = motorY; secondaryMotor = motorX;
    primaryMotorStepsTotal = abs(dy); secondaryMotorStepsTotal = abs(dx);
    primaryMotorDirection = dirY; secondaryMotorDirection = dirX;
    mainAxisIsX = false; // для расчета currentX currentY при выполнение шага, для обратного перевода с первичного мотора на осевой, не забыть мне!
    }

    if (primaryMotorStepsTotal == 0) { // пустое задание
        return;
    }

    // собственно стави задачу моторам шагать заданое количество шагов
    primaryMotorStepsRemaining = primaryMotorStepsTotal;
    secondaryMotorStepsRemaining = secondaryMotorStepsTotal;
    primaryMotorStepsDone = 0;
    secondaryMotorStepsDone = 0;

    // пока приблизительно посчитаем скорость - затем будет апдейтер ее обновлять
    uint32_t primaryInterval = (uint32_t)ceil(1e6f / tool_curise_speed);
    uint32_t secondaryInterval = 0;
    float ratio = (float)(secondaryMotorStepsTotal) / primaryMotorStepsTotal;

    while (true) {
        float secIntF = (float)primaryInterval / ratio;
        uint32_t secInt = (uint32_t)(secIntF + 0.99999f);
        float testRatio = (float)primaryInterval / (float)secInt;
        if (fabs(testRatio - ratio) < 1e-5) {
        secondaryInterval = secInt;
        break;
        }
        primaryInterval++;
    }

    primaryMotorCurrentStepIntervalUs = primaryInterval; 
    secondaryMotorCurrentStepIntervalUs = secondaryInterval;

}


void StepperMotorController::handlePlaner(){
    // расчитываем интервалы согласно текущему профилю и корректируем ошибки (догоняем, притормаживаем)
    if (currentX_printed!=currentX || currentY_printed != currentY){
        Serial.print("New current position: x=");Serial.print(currentX);Serial.print(" y=");Serial.print(currentY);Serial.print("\n");
        currentX_printed = currentX;
        currentY_printed = currentY;
    }
}

void StepperMotorController::handleMotors(){
    absolute_time_t now = get_absolute_time();
    stepsMotorPosition(now);
    stepsMotorSpeeds(now);
    finalizeMotorsSteps(now);
}