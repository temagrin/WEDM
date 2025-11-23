// motor_planner.cpp
#include "motor_planner.h"


StepperMotorPlanner::StepperMotorPlanner(StepperMotorController* controller)
    : controller(controller){}


bool StepperMotorPlanner::moveTo(int64_t newX, int64_t newY, uint32_t tool_speed) { // постановка задачи
    return true;
}


void StepperMotorPlanner::handlePlaner(){ // вызывается часто
    
}


void StepperMotorPlanner::reset(){
}

bool StepperMotorPlanner::ready(){
    return true;
}
 