#include "xy_stepper_planner.h"
#include "pico/time.h"
#include <cstdlib>

XYStepperPlanner::XYStepperPlanner(StepperMotor& xMotor, StepperMotor& yMotor)
    : motorX(xMotor), motorY(yMotor) {}

void XYStepperPlanner::moveTo(int32_t target_x, int32_t target_y, uint32_t total_time_us) {
    if (total_time_us == 0) return;  
    deltaX = target_x - currentX;
    deltaY = target_y - currentY;
    targetX = target_x;
    targetY = target_y;
    this->total_time_us = total_time_us;
    moving = true;

    startMotion();
}

void XYStepperPlanner::startMotion() {
    int32_t stepsX = std::abs(deltaX);
    int32_t stepsY = std::abs(deltaY);
    if (stepsX == 0 && stepsY == 0) {
        moving = false;
        motorX.stopMove();
        motorY.stopMove();
        return;
    }

    uint32_t interval_x = (stepsX == 0) ? 0 : total_time_us / stepsX;
    uint32_t interval_y = (stepsY == 0) ? 0 : total_time_us / stepsY;

    motorX.startMoveSteps(deltaX, interval_x * stepsX);
    motorY.startMoveSteps(deltaY, interval_y * stepsY);
}

void XYStepperPlanner::update() {
    if (!moving) return;

    int32_t remX = motorX.remainingSteps();
    int32_t remY = motorY.remainingSteps();

    if (remX < 0) remX = 0;
    if (remY < 0) remY = 0;

    currentX = targetX - remX * (deltaX >= 0 ? 1 : -1);
    currentY = targetY - remY * (deltaY >= 0 ? 1 : -1);

    if (remX == 0 && remY == 0) {
        moving = false;
        motorX.stopMove();
        motorY.stopMove();
    }
}

void XYStepperPlanner::getPosition(int32_t& x_out, int32_t& y_out) const {
    x_out = currentX;
    y_out = currentY;
}
