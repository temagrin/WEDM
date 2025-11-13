#ifndef XY_STEPPER_PLANNER_H
#define XY_STEPPER_PLANNER_H

#include "motor.h"
#include <cstdint>

class XYStepperPlanner {
public:
    XYStepperPlanner(StepperMotor& xMotor, StepperMotor& yMotor);
    void moveTo(int32_t target_x, int32_t target_y, uint32_t total_time_us);
    void update();
    void getPosition(int32_t& x_out, int32_t& y_out) const;

private:
    StepperMotor& motorX;
    StepperMotor& motorY;

    int32_t currentX = 0;
    int32_t currentY = 0;

    int32_t targetX = 0;
    int32_t targetY = 0;

    int32_t deltaX = 0;
    int32_t deltaY = 0;

    uint32_t total_time_us = 0;

    bool moving = false;

    void startMotion();

};

#endif
