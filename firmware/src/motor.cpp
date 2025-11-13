// motor.cpp
#include "motor.h"
#include <cmath>

StepperMotor::StepperMotor(uint step_pin, uint dir_pin, uint en_pin) : pins{step_pin, dir_pin, en_pin} {}

void StepperMotor::init() {
    gpio_init(pins.gpio_step);
    gpio_set_dir(pins.gpio_step, GPIO_OUT);

    gpio_init(pins.gpio_dir);
    gpio_set_dir(pins.gpio_dir, GPIO_OUT);

    gpio_init(pins.gpio_en);
    gpio_set_dir(pins.gpio_en, GPIO_OUT);

    gpio_put(pins.gpio_en, ENABLE_PIN_LEVEL);
    enabled = true;
    cmd.type = MotorCommand::Type::NONE;
}

void StepperMotor::doStep() {
    if (!enabled) return;
    gpio_put(pins.gpio_dir, dir ? 1 : 0);
    gpio_put(pins.gpio_step, 1);
    sleep_us(2);
    gpio_put(pins.gpio_step, 0);
}

void StepperMotor::handle() {
    absolute_time_t now = get_absolute_time();
    if (cmd.type == MotorCommand::Type::NONE) return;
    uint32_t dt = absolute_time_diff_us(cmd.last_step_time, now);
    if (dt < cmd.interval_us) return;

    if (cmd.type == MotorCommand::Type::MOVE_N_STEPS) {
        if (cmd.remaining_steps <= 0) {
            cmd.type = MotorCommand::Type::NONE;
            return;
        }
        doStep();
        cmd.remaining_steps--;
        steps_done++;
        cmd.last_step_time = now;
    } else if (cmd.type == MotorCommand::Type::CONSTANT_SPEED) {
        if (cmd.interval_us == 0) return;
        doStep();
        steps_done++;
        cmd.last_step_time = now;
    }
}

void StepperMotor::setConstantSpeed(int32_t speed) {
    if (speed == 0) {
        cmd.type = MotorCommand::Type::NONE;
        cmd.interval_us = 0;
        return;
    }
    dir = (speed > 0);
    cmd.type = MotorCommand::Type::CONSTANT_SPEED;
    cmd.interval_us = static_cast<uint32_t>(std::abs(speed));
    cmd.last_step_time = get_absolute_time();
}

void StepperMotor::startMoveSteps(int32_t steps, uint32_t total_time_us) {
    if (steps == 0) {
        cmd.type = MotorCommand::Type::NONE;
        return;
    }
    dir = (steps > 0);
    uint32_t abs_steps = (steps > 0) ? static_cast<uint32_t>(steps) : static_cast<uint32_t>(-steps);
    cmd.type = MotorCommand::Type::MOVE_N_STEPS;
    cmd.remaining_steps = abs_steps;
    cmd.interval_us = total_time_us / abs_steps;
    cmd.last_step_time = get_absolute_time();
    steps_done = 0;
}

int32_t StepperMotor::adjustMoveTime(uint32_t new_time_us) {
    if (cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;
    if (cmd.remaining_steps <= 0) {
        cmd.type = MotorCommand::Type::NONE;
        return 0;
    }
    cmd.interval_us = new_time_us / cmd.remaining_steps;
    return cmd.remaining_steps;
}

int32_t StepperMotor::stopMove() {
    if (cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;
    int32_t rem = cmd.remaining_steps;
    cmd.remaining_steps = 0;
    cmd.type = MotorCommand::Type::NONE;
    return rem;
}

int32_t StepperMotor::remainingSteps() const {
    if (cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;
    return cmd.remaining_steps;
}


// --- StepperMotorController implementation ---

StepperMotorController::StepperMotorController() : motor_count(0) {}

bool StepperMotorController::addMotor(StepperMotor& motor) {
    if (motor_count >= MAX_MOTORS) return false;
    motors[motor_count++] = &motor;
    return true;
}

void StepperMotorController::init() {
    for (int i = 0; i < motor_count; i++) {
        if (motors[i]) motors[i]->init();
    }
}

void StepperMotorController::handleMotors() {
    for (int i = 0; i < motor_count; i++) {
        if (motors[i]) motors[i]->handle();
    }
}
