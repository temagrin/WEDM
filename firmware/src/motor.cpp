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
}

void StepperMotor::doStep(bool dir, absolute_time_t now) {
    step_onset_time = now;
    gpio_put(config.dir_pin, dir ^ config.invert_dir_pin);
    gpio_put(config.step_pin, !config.invert_step_pin);
}

void StepperMotor::afterStep() {
    step_onset_time = 0;
    gpio_put(config.step_pin, config.invert_step_pin);
}

void StepperMotor::setPower(bool en) {
    gpio_put(config.en_pin, en ^ config.invert_en_pin);
}

absolute_time_t StepperMotor::getStepOnsetTime() const {
    return step_onset_time;
}

uint8_t StepperMotor::getStepPulseDuration() const {
    return config.step_pulse_duration;
}
