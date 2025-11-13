// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#define ENABLE_PIN_LEVEL 0 // 0 для LV8729, 1 для A4988

#include <cstdint>
#include "pico/stdlib.h"

class StepperMotor {
public:
    struct StepperPins {
        uint gpio_step;
        uint gpio_dir;
        uint gpio_en;
    };

    explicit StepperMotor(uint step_pin, uint dir_pin, uint en_pin);

    void init();

    // Интерфейс управления мотором
    void handle();
    void setConstantSpeed(int32_t speed);
    void startMoveSteps(int32_t steps, uint32_t total_time_us);
    int32_t adjustMoveTime(uint32_t new_time_us);
    int32_t stopMove();
    int32_t remainingSteps() const;

private:
    StepperPins pins;

    struct MotorCommand {
        enum class Type { CONSTANT_SPEED, MOVE_N_STEPS, NONE } type = Type::NONE;
        int32_t remaining_steps = 0;
        uint32_t interval_us = 0;
        absolute_time_t last_step_time;
    };

    MotorCommand cmd;
    bool dir = false;
    bool enabled = false;
    int32_t steps_done = 0;

    void doStep();
};

class StepperMotorController {
public:
    static constexpr int MAX_MOTORS = 8;

    StepperMotorController();

    bool addMotor(StepperMotor& motor);  // Добавляет мотор, возвращает false, если превышен лимит

    void init();
    void handleMotors();

private:
    StepperMotor* motors[MAX_MOTORS] = {nullptr};
    int motor_count = 0;
};

#endif