// motor_planner.h
#ifndef MOTORPLANNER_H
#define MOTORPLANNER_H

#include <cmath>
#include "motor_controller.h"



class StepperMotorPlanner {
public:
    StepperMotorPlanner(StepperMotorController* controller);
    void handlePlaner();
    bool moveTo(int64_t newX, int64_t newY, uint32_t tool_speed);
    void reset(); // сбросить координаты для тестов
    bool ready(); 


private:
    StepperMotorController* controller;

};

#endif // MOTORPLANNER_H