// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
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
enum G_AXIS {G_AXIS_XY, G_AXIS_A, G_AXIS_B, G_AXIS_COUNT};


static constexpr uint AXIS_DIRECTION_PIN [AXIS_COUNT] = {DIR_X_PIN, DIR_Y_PIN, DIR_A_PIN, DIR_B_PIN};
static constexpr uint AXIS_DIRECTION_INVERT [AXIS_COUNT] = {MOTOR_X_DIR_INVERT, MOTOR_Y_DIR_INVERT, MOTOR_A_DIR_INVERT, MOTOR_B_DIR_INVERT};


struct MotorBresenhamState {
    int32_t targetX;
    int32_t targetY;
    int32_t taskCurrentX;
    int32_t taskCurrentY;
    int32_t errorPosition;
    bool finished;
};
struct MotorDirectionState {
    bool direction;
};

struct MotorSpeedState {
    absolute_time_t stepLastTime;
    uint32_t stepInterval;
    uint32_t errorIncrement;
    uint32_t errorAccumulator;
};


class StepperMotorController {
public:
    StepperMotorController();

    static void initMotors();
    void tick(absolute_time_t now);

    void setWaitingSpeed(G_AXIS axis, uint32_t stepInterval, uint32_t errorIncrement);
    void setWaitingSteps(uint32_t stepsX, uint32_t stepsY);
    void setWaitingDirection(AXIS axis, bool direction);


    bool addToBuffer(uint8_t ctrlFlags, uint32_t stepsX, uint32_t stepsY,
                     uint32_t intSpeedPart, uint32_t errorIncrement);

    void setBreakFactor(uint8_t value);

    size_t getQueueAvailable() { return queue.available();}
    static bool powerControl(uint8_t ctrlFlags);
    bool resetPosition(){currentPositionX=0; currentPositionY=0; return true;}
    [[nodiscard]] int32_t getCurrentPositionX() const {return currentPositionX;}
    [[nodiscard]] int32_t getCurrentPositionY() const {return currentPositionY;}


private:
    MotorBresenhamState flipStepState, flopStepState;

    MotorSpeedState flipSpeedStateXY, flopSpeedStateXY;
    MotorSpeedState flipSpeedStateA, flopSpeedStateA;
    MotorSpeedState flipSpeedStateB, flopSpeedStateB;

    MotorDirectionState flipDirectionStateX, flopDirectionStateX;
    MotorDirectionState flipDirectionStateY, flopDirectionStateY;
    MotorDirectionState flipDirectionStateA, flopDirectionStateA;
    MotorDirectionState flipDirectionStateB, flopDirectionStateB;

    MotorBresenhamState* activeStepState;
    MotorBresenhamState* waitingStepState;

    MotorSpeedState* activeSpeedStates[G_AXIS_COUNT];
    MotorSpeedState* waitingSpeedStates[G_AXIS_COUNT];

    MotorDirectionState* activeDirectionStates[AXIS_COUNT];
    MotorDirectionState* waitingDirectionStates[AXIS_COUNT];

    CommandRingBuffer queue = CommandRingBuffer();

    volatile int32_t currentPositionX = 0;
    volatile int32_t currentPositionY = 0;

    void setIntervalA(bool direction, uint32_t intSpeedPart, uint32_t errorIncrement);
    void setIntervalB(bool direction, uint32_t intSpeedPart, uint32_t errorIncrement);

    bool needStep(G_AXIS axis, absolute_time_t now);
    uint8_t needBresenhamSteps();
    void applyStepsToCurrentPosition(uint8_t stepsMask);
    static void doSteps(uint8_t stepMask);
    bool breakStepXY();
    volatile uint8_t breakFactor = 0;
    uint8_t stepSkipCounter = 0;

    void popXYStates();

    void flipFlopSpeedState(G_AXIS axis); // индивидуально по группам осей
    void flipFlopDirectionState(AXIS axis); // индивидуально по осям
    void flipFlopStepState();

    static void initPin(uint16_t _pin, bool defaultValue);
    static void setPowerXY(const bool value){gpio_put(EN_X_Y_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerA(const bool value){gpio_put(EN_A_PIN, MOTOR_ENABLE_INVERT ^ value);}
    static void setPowerB(const bool value){gpio_put(EN_B_PIN, MOTOR_ENABLE_INVERT ^ value);}

};


#endif // MOTORCONTROLLER_H