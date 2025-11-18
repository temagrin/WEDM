// motor.cpp
#include "motor.h"

StepperMotor::StepperMotor(
    uint8_t step_pin,
    uint8_t dir_pin,
    uint8_t en_pin,
    const MotorDriver& driver_settings
)
    : config{
        step_pin, dir_pin, en_pin,
        driver_settings.invert_step_pin,
        driver_settings.invert_dir_pin,
        driver_settings.invert_en_pin,
        driver_settings.step_pulse_duration
    }
{}


void StepperMotor::init() {
    // обозначаем пины на выход
    gpio_init(config.step_pin);
    gpio_set_dir(config.step_pin, GPIO_OUT);
    gpio_init(config.dir_pin);
    gpio_set_dir(config.dir_pin, GPIO_OUT);
    gpio_init(config.en_pin);
    gpio_set_dir(config.en_pin, GPIO_OUT);
    // ставим дефолтное состояние из конфигурации инверсий
    gpio_put(config.step_pin, config.invert_step_pin);
    gpio_put(config.dir_pin, config.invert_dir_pin);
    gpio_put(config.en_pin, config.invert_en_pin);

    motorState.calcSteps = false;
    motorState.direction = false;
    motorState.stepInterval = 0;
    motorState.stepLastTime = get_absolute_time();
    motorState.stepOnsetTime = 0;
    motorState.stepsDone = 0;
    motorState.stepsRemaining = 0;

}

void StepperMotor::setPower(bool power) { 
    gpio_put(config.en_pin, power ^ config.invert_en_pin); 
}
    

void StepperMotor::doStep(absolute_time_t now) {
    motorState.stepOnsetTime = now;
    motorState.stepLastTime = now;
    gpio_put(config.dir_pin, motorState.direction ^ config.invert_dir_pin);
    gpio_put(config.step_pin, !config.invert_step_pin);
}

void StepperMotor::afterStep() {
    motorState.stepOnsetTime = 0;
    gpio_put(config.step_pin, config.invert_step_pin);
}

void StepperMotor::considStep(){
    motorState.stepsRemaining--;
    motorState.stepsDone++;
    motorState.direction ? motorState.absoluteSteps++ : motorState.absoluteSteps--;
}

void StepperMotor::setInfininityRorationSpeed(int32_t speed){
    Serial.println("Constant speed rotation");
    motorState.calcSteps = false;
    if (speed == 0){
        motorState.stepInterval = 0;
        motorState.running = false;
    } else {
        motorState.direction = speed>0;
        motorState.stepInterval = (uint32_t)(1e6f / abs(speed));
        motorState.running = true;
    }
    Serial.println(motorState.stepInterval);
    Serial.println();  
}