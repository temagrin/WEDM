// motor_controller.cpp
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

// завершить шаг для моторов у которых прошло нужное время и был установлен новый уровень
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

void StepperMotorController::finalizeMotors(absolute_time_t now) {
    finalizeStep(motorX, now);
    finalizeStep(motorY, now);
    for (uint8_t i=0; i<motor_count; i++) {
        finalizeStep(motors[i], now);
    }    
}


void StepperMotorController::handleMotors(){
    absolute_time_t now = get_absolute_time();
    int64_t dt  = absolute_time_diff_us(last_handle_time, now);
    last_handle_time = now;
    finalizeMotors(now);
    

}