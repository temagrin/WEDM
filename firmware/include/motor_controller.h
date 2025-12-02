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
enum G_AXIS {G_AXIS_XY, A_AXIS_A, G_AXIS_B, G_AXIS_COUNT};


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

    size_t getQueueAvailable() { return queue.available();}
    static bool powerControl(uint8_t ctrlFlags);
    bool resetPosition(){currentPositionX=0; currentPositionY=0; return true;}
    [[nodiscard]] int32_t getCurrentPositionX() const {return currentPositionX;}
    [[nodiscard]] int32_t getCurrentPositionY() const {return currentPositionY;}


private:
    // TODO проинитить это все внимательно!
    MotorBresenhamState flipStepState{}, flopStepState{};
    MotorBresenhamState* activeStepState = nullptr;
    MotorBresenhamState* waitingStepState = nullptr;

    MotorSpeedState flipSpeedStateXY{}, flopSpeedStateXY{};
    MotorSpeedState flipSpeedStateA{}, flopSpeedStateA{};
    MotorSpeedState flipSpeedStateB{}, flopSpeedStateB{};

    MotorSpeedState* activeSpeedStateXY = nullptr;
    MotorSpeedState* waitingSpeedStateXY = nullptr;
    MotorSpeedState* activeSpeedStateA = nullptr;
    MotorSpeedState* waitingSpeedStateA = nullptr;
    MotorSpeedState* activeSpeedStateB = nullptr;
    MotorSpeedState* waitingSpeedStateB = nullptr;
    MotorSpeedState** activeSpeedStates[G_AXIS_COUNT] = {};
    MotorSpeedState** waitingSpeedStates[G_AXIS_COUNT] = {};

    MotorDirectionState flipDirectionStateX{}, flopDirectionStateX{};
    MotorDirectionState flipDirectionStateY{}, flopDirectionStateY{};
    MotorDirectionState flipDirectionStateA{}, flopDirectionStateA{};
    MotorDirectionState flipDirectionStateB{}, flopDirectionStateB{};

    MotorDirectionState* activeDirectionStateX = nullptr;
    MotorDirectionState* waitingDirectionStateX = nullptr;
    MotorDirectionState* activeDirectionStateY = nullptr;
    MotorDirectionState* waitingDirectionStateY = nullptr;
    MotorDirectionState* activeDirectionStateA = nullptr;
    MotorDirectionState* waitingDirectionStateA = nullptr;
    MotorDirectionState* activeDirectionStateB = nullptr;
    MotorDirectionState* waitingDirectionStateB = nullptr;

    MotorDirectionState** activeDirectionStates[AXIS_COUNT] = {};
    MotorDirectionState** waitingDirectionStates[AXIS_COUNT] = {};

    CommandRingBuffer queue = {};

    volatile int32_t currentPositionX = 0;
    volatile int32_t currentPositionY = 0;


    bool needStep(G_AXIS axis, absolute_time_t now);
    uint8_t needBresenhamSteps();
    void applyStepsToCurrentPosition(uint8_t stepsMask);
    static void doSteps(uint8_t stepMask);


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