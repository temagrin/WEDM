// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include <atomic>
#include <fastmath.h>
#include "hw_config.h"
#include "stepper.pio.h"
#include "pico/multicore.h"
#include "ring_buffer.h"

#if MOTOR_STEP_INVERT
  #define STEPS_TEMPLATE 0xF0 // 11110000
#else
  #define STEPS_TEMPLATE 0x00 // 00000000
#endif


enum AXIS {AXIS_X, AXIS_Y, AXIS_A, AXIS_B, AXIS_COUNT};
static constexpr uint AXIS_DIRECTION_PIN [AXIS_COUNT] = {DIR_X_PIN, DIR_Y_PIN, DIR_A_PIN, DIR_B_PIN};
static constexpr uint AXIS_DIRECTION_INVERT [AXIS_COUNT] = {MOTOR_X_DIR_INVERT, MOTOR_Y_DIR_INVERT, MOTOR_A_DIR_INVERT, MOTOR_B_DIR_INVERT};
#define READY_XY 0x03
#define READY_A 0x04
#define READY_B 0x08


struct MotorStepState {
    absolute_time_t stepLastTime;
    bool direction;
    bool positionMode;
    uint32_t stepsRemaining;
};

struct MotorSpeedState {
    uint32_t stepInterval;
    uint32_t errorIncrement;
    uint32_t errorAccumulator;
};


class StepperMotorController {
public:
    explicit StepperMotorController(CommandRingBuffer &queue_);
    static void initMotors();
    void tick(absolute_time_t now);
    void setWaitingSpeed(AXIS axis, uint32_t stepInterval, uint32_t errorIncrement);
    void setWaitingSteps(AXIS axis, uint32_t steps, bool direction, bool positionMode);
    static bool powerControl(uint8_t ctrlFlags);
    bool resetPosition(){currentPositionX=0; currentPositionY=0; return true;}
    bool addToBuffer(uint8_t ctrlFlags,
                     uint32_t stepsX, uint32_t stepsY,
                     uint32_t intSpeedPartX, uint32_t intSpeedPartY,
                     uint32_t errorIncrementX, uint32_t errorIncrementY);
    size_t getQueueAvailable() { return queue.available();}
    [[nodiscard]] int32_t getCurrentPositionX() const {return currentPositionX;}
    [[nodiscard]] int32_t getCurrentPositionY() const {return currentPositionY;}


private:
    MotorSpeedState flipSpeedStateX, flopSpeedStateX;
    MotorSpeedState flipSpeedStateY, flopSpeedStateY;
    MotorSpeedState flipSpeedStateA, flopSpeedStateA;
    MotorSpeedState flipSpeedStateB, flopSpeedStateB;

    MotorStepState flipStepStateX, flopStepStateX;
    MotorStepState flipStepStateY, flopStepStateY;
    MotorStepState flipStepStateA, flopStepStateA;
    MotorStepState flipStepStateB, flopStepStateB;

    MotorSpeedState* activeSpeedStateX;
    MotorSpeedState* waitingSpeedStateX;
    MotorSpeedState* activeSpeedStateY;
    MotorSpeedState* waitingSpeedStateY;
    MotorSpeedState* activeSpeedStateA;
    MotorSpeedState* waitingSpeedStateA;
    MotorSpeedState* activeSpeedStateB;
    MotorSpeedState* waitingSpeedStateB;
    MotorSpeedState** activeSpeedStates[AXIS_COUNT];
    MotorSpeedState** waitingSpeedStates[AXIS_COUNT];

    MotorStepState* activeStepStateX;
    MotorStepState* waitingStepStateX;
    MotorStepState* activeStepStateY;
    MotorStepState* waitingStepStateY;
    MotorStepState* activeStepStateA;
    MotorStepState* waitingStepStateA;
    MotorStepState* activeStepStateB;
    MotorStepState* waitingStepStateB;
    MotorStepState** activeStepStates[AXIS_COUNT];
    MotorStepState** waitingStepStates[AXIS_COUNT];

    volatile int32_t currentPositionX;
    volatile int32_t currentPositionY;
    CommandRingBuffer& queue;

    bool needStep(AXIS axis, absolute_time_t now);
    static void doSteps(uint8_t stepMask);
    volatile bool lastDirection[AXIS_COUNT] = {false, false, false, false};

    uint8_t readyMask = 0;

    static void initPin(uint16_t _pin, bool defaultValue);
    void adjustDirections();
    void flipFlopSpeedState(AXIS axis);
    void flipFlopStepState(AXIS axis);

    static void setPowerXY(const bool value){gpio_put(EN_X_Y_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerA(const bool value){gpio_put(EN_A_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerB(const bool value){gpio_put(EN_B_PIN, MOTOR_ENABLE_INVERT ^ value);}
    bool popXYStates();
    bool popAStates();
    bool popBStates();
};


#endif // MOTORCONTROLLER_H