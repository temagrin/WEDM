// motor_controller.h
#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include <fastmath.h>
#include <cstdio>
#include <hardware/pio.h>
#include "stepper.pio.h" 
#include "hw_config.h"



#if MOTOR_STEP_INVERT
  #define STEPS_TEMPLATE 0xF0 // 11110000
#else
  #define STEPS_TEMPLATE 0x00 // 00000000
#endif


static constexpr uint32_t SCALE = 1000000000;
static constexpr double DSCALE = (double)SCALE;

class StepperMotorController {
    struct MotorState {
        absolute_time_t stepLastTime;
        uint32_t stepInterval;
        uint32_t errorIncrement;
        uint32_t errorAccumulator;
        bool direction;
        bool positionMode;
    };

public:
    StepperMotorController();
    void initMotors();
    void tick(absolute_time_t now);
    void setSpeed(MotorState* state, int32_t speed);
    inline void setSpeedX(int32_t speed){setSpeed(&stateX, speed);setDirX(stateX.direction);}
    inline void setSpeedY(int32_t speed){setSpeed(&stateY, speed);setDirY(stateY.direction);}
    inline void setSpeedA(int32_t speed){setSpeed(&stateA, speed);setDirA(stateA.direction);}
    inline void setSpeedB(int32_t speed){setSpeed(&stateB, speed);setDirB(stateB.direction);}
    
    inline void setDirX(bool value){gpio_put(DIR_X_PIN, MOTOR_X_DIR_INVERT ^ value);}
    inline void setDirY(bool value){gpio_put(DIR_Y_PIN, MOTOR_Y_DIR_INVERT ^ value);}
    inline void setDirA(bool value){gpio_put(DIR_A_PIN, MOTOR_A_DIR_INVERT ^ value);}
    inline void setDirB(bool value){gpio_put(DIR_B_PIN, MOTOR_B_DIR_INVERT ^ value);}
    inline void setPowerXY(bool value){gpio_put(EN_X_Y_PIN, MOTOR_ENABLE_INVERT ^ value);}
    inline void setPowerA(bool value){gpio_put(EN_A_PIN, MOTOR_ENABLE_INVERT ^ value);}
    inline void setPowerB(bool value){gpio_put(EN_B_PIN, MOTOR_ENABLE_INVERT ^ value);}



private:
    void initPin(uint16_t _pin, bool defaultValue);
    void doSteps(uint8_t doStepMask);
    bool needStep(MotorState* state, absolute_time_t now);
    MotorState stateX;
    MotorState stateY;
    MotorState stateA;
    MotorState stateB;
};

#endif // MOTORCONTROLLER_H