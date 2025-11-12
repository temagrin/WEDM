#include <array>
#include <cstdint>
#include <cstdio>
#include <math.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "hw_config.h"

std::array<MotorState, MOTOR_COUNT> motors;

void init_motors() {
    motors[0].pins = {STEP_1_PIN, DIR_1_PIN, EN_1_PIN};
    motors[1].pins = {STEP_2_PIN, DIR_2_PIN, EN_2_PIN};
    motors[2].pins = {STEP_3_PIN, DIR_3_PIN, EN_3_PIN};
    motors[3].pins = {STEP_4_PIN, DIR_4_PIN, EN_4_PIN};

    for(auto& m : motors) {
        gpio_init(m.pins.gpio_step);
        gpio_set_dir(m.pins.gpio_step, GPIO_OUT);
        gpio_init(m.pins.gpio_dir);
        gpio_set_dir(m.pins.gpio_dir, GPIO_OUT);
        gpio_init(m.pins.gpio_en);
        gpio_set_dir(m.pins.gpio_en, GPIO_OUT);
        gpio_put(m.pins.gpio_en, ENABLE_PIN_LEVEL);
        m.enabled = true;
        m.cmd.type = MotorCommand::Type::NONE;
    }
}

void do_step(MotorState &motor) {
    if(!motor.enabled) return;
    gpio_put(motor.pins.gpio_dir, motor.dir ? 1 : 0);
    gpio_put(motor.pins.gpio_step, 1);
    sleep_us(2);
    gpio_put(motor.pins.gpio_step, 0);
}

void handle_motors() {
    absolute_time_t now = get_absolute_time();
    for(auto& motor : motors) {
        if(motor.cmd.type == MotorCommand::Type::NONE) continue;
        uint32_t dt = absolute_time_diff_us(motor.cmd.last_step_time, now);
        if(dt < motor.cmd.interval_us) continue;
        // Для MOVE_N_STEPS считаем остаток
        if(motor.cmd.type == MotorCommand::Type::MOVE_N_STEPS) {
            if(motor.cmd.remaining_steps <= 0) {
                motor.cmd.type = MotorCommand::Type::NONE;
                continue;
            }
            do_step(motor);
            motor.cmd.remaining_steps--;
            motor.steps_done++;
            motor.cmd.last_step_time = now;
        } else if(motor.cmd.type == MotorCommand::Type::CONSTANT_SPEED) {
            if (motor.cmd.interval_us == 0) continue;
            do_step(motor);
            motor.steps_done++;
            motor.cmd.last_step_time = now;
        }
    }
}


int32_t adjust_move_time(int motor_index, uint32_t new_time_us) {
    if(motor_index < 0 || motor_index >= MOTOR_COUNT) return -1;
    MotorState &m = motors[motor_index];
    if(m.cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;

    // Пересчитаем интервал чтобы провести остаток шагов за new_time_us
    if(m.cmd.remaining_steps <= 0) {
        m.cmd.type = MotorCommand::Type::NONE;
        return 0;
    }
    m.cmd.interval_us = new_time_us / m.cmd.remaining_steps;
    return m.cmd.remaining_steps;
}

int32_t remaining_steps(int motor_index) {
    if(motor_index < 0 || motor_index >= MOTOR_COUNT) return -1;
    MotorState &m = motors[motor_index];
    if(m.cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;
    return m.cmd.remaining_steps;
}


int32_t stop_move(int motor_index) {
    if(motor_index < 0 || motor_index >= MOTOR_COUNT) return -1;
    MotorState &m = motors[motor_index];
    if(m.cmd.type != MotorCommand::Type::MOVE_N_STEPS) return -1;
    int32_t remaining = m.cmd.remaining_steps;
    m.cmd.remaining_steps = 0;
    m.cmd.type = MotorCommand::Type::NONE;
    return remaining;
}

void set_constant_speed(int motor_index, int32_t speed) {
    if (motor_index < 0 || motor_index >= MOTOR_COUNT) return;
    MotorState &m = motors[motor_index];
    if (speed == 0) {
        m.cmd.type = MotorCommand::Type::NONE;
        m.cmd.interval_us = 0;
        return;
    }
    m.dir = (speed > 0);
    m.cmd.type = MotorCommand::Type::CONSTANT_SPEED;
    m.cmd.interval_us = (uint32_t)abs(speed);
    m.cmd.last_step_time = get_absolute_time();
}

// Запуск движения N шагов за время T
void start_move_steps(int motor_index, int32_t steps, uint32_t total_time_us) {
    if(motor_index < 0 || motor_index >= MOTOR_COUNT) return;
    MotorState &m = motors[motor_index];
    if(steps == 0) {
        m.cmd.type = MotorCommand::Type::NONE;
        return;
    }
    m.dir = (steps > 0);
    uint32_t abs_steps = (steps > 0) ? steps : -steps;
    m.cmd.type = MotorCommand::Type::MOVE_N_STEPS;
    m.cmd.remaining_steps = abs_steps;
    m.cmd.interval_us = total_time_us / abs_steps;
    m.cmd.last_step_time = get_absolute_time();
    m.steps_done = 0;
}

