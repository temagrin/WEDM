#ifndef MOTOR_H
#define MOTOR_H

#include <cstdint>
#include "pico/stdlib.h"

struct StepperPins {
    uint gpio_step;
    uint gpio_dir;
    uint gpio_en;
};

constexpr int MOTOR_COUNT = 4;

struct MotorCommand {
    enum class Type { CONSTANT_SPEED, MOVE_N_STEPS, NONE } type = Type::NONE;
    int32_t remaining_steps = 0;
    uint32_t interval_us = 0;
    absolute_time_t last_step_time;
};

struct MotorState {
    StepperPins pins;
    MotorCommand cmd;
    bool dir = false;
    bool enabled = false;
    int32_t steps_done = 0;
};



void do_step(MotorState &motor);

void handle_motors();

int32_t adjust_move_time(int motor_index, uint32_t new_time_us);

int32_t stop_move(int motor_index);

void set_constant_speed(int motor_index, int32_t speed);

void start_move_steps(int motor_index, int32_t steps, uint32_t total_time_us);
int32_t remaining_steps(int motor_index);

void init_motors();

#endif