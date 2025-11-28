// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include "stepper.pio.h"
#include <pico/time.h>
#include "hw_config.h"
#include "ring_buffer.h"


#if MOTOR_STEP_INVERT
  #define STEPS_TEMPLATE 0xF0 // 11110000
#else
  #define STEPS_TEMPLATE 0x00 // 00000000
#endif


static constexpr uint32_t SCALE = 1000000000;
static constexpr double DOUBLE_SCALE = SCALE;


class StepperMotorController {
    struct MotorState {
        absolute_time_t stepLastTime;
        volatile uint32_t stepInterval;
        volatile uint32_t errorIncrement;
        uint32_t errorAccumulator;
        volatile bool direction;
        bool positionMode;
        volatile uint32_t steps_remaining;
    };

public:
    StepperMotorController(const CommandRingBuffer& q);
    void initMotors();
    void tick(absolute_time_t now);
    void checkBuffer();

    static void setSpeed(MotorState* state, double speed);

    void setSpeedX(const double speed){setSpeed(&stateX, speed);setDirX(stateX.direction);}
    void setSpeedY(const double speed){setSpeed(&stateY, speed);setDirY(stateY.direction);}
    void setSpeedA(const double speed){setSpeed(&stateA, speed);setDirA(stateA.direction);}
    void setSpeedB(const double speed){setSpeed(&stateB, speed);setDirB(stateB.direction);}

    static void setDirX(const bool value){gpio_put(DIR_X_PIN, MOTOR_X_DIR_INVERT ^ value);}
    static void setDirY(const bool value){gpio_put(DIR_Y_PIN, MOTOR_Y_DIR_INVERT ^ value);}
    static void setDirA(const bool value){gpio_put(DIR_A_PIN, MOTOR_A_DIR_INVERT ^ value);}
    static void setDirB(const bool value){gpio_put(DIR_B_PIN, MOTOR_B_DIR_INVERT ^ value);}
    static void setPowerXY(const bool value){gpio_put(EN_X_Y_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerA(const bool value){gpio_put(EN_A_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerB(const bool value){gpio_put(EN_B_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static bool powerMotors(uint8_t ctrl_flags);
    [[nodiscard]] int32_t getCurrentPositionX() const{return currentPositionX;}
    [[nodiscard]] int32_t getCurrentPositionY() const{return currentPositionY;}
    void setBreak(uint8_t value); // 0 - не тормозим, 255 - полная остановка

private:
    CommandRingBuffer queue;
    int32_t currentPositionX;
    int32_t currentPositionY;
    static void initPin(uint16_t _pin, bool defaultValue);
    static void doSteps(uint8_t doStepMask);
    static bool needStep(MotorState* state, absolute_time_t now);
    MotorState stateX;
    MotorState stateY;
    MotorState stateA;
    MotorState stateB;
    volatile uint8_t oldBreakValue;
    volatile double breakFactor;
    volatile double originalSpeedX;
    volatile double originalSpeedY;
};

#endif // MOTORCONTROLLER_H