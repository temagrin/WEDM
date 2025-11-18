// motor_planner.h
#ifndef MOTORPLANNER_H
#define MOTORPLANNER_H

#include <SerialUSB.h>
#include <cmath>
#include "motor_controller.h"



class StepperMotorPlanner {
public:
    StepperMotorPlanner(StepperMotor* motorX, StepperMotor* motorY);
    void handlePlaner();
    bool moveTo(int64_t newX, int64_t newY, uint32_t tool_speed);
    void reset(); // сбросить координаты для тестов
    bool ready(); 

    static constexpr double deltaT = 1.0;  // минимальная единица коррекции, 1 мкс
    static constexpr double minAllowedAlignmentInterval = 50.0; // минимальный интервал вызова коррекции (мкс)
    static constexpr double maxAllowedStepInterval = 1e8; // максимальный интервал шага (10 секунд)
    static constexpr double minAllowedStepInterval = 10.0; // минимальный интервал шага (10 мкс)
    static constexpr double maxAccumulatedError = 0.5; // максимально допустимый размах накопленной ошибки (в мкс)

private:
    StepperMotor* motorX; 
    StepperMotor* motorY;    
    int64_t currentX=0;
    int64_t currentY=0;
    uint32_t xStepInterval = 0;
    uint32_t yStepInterval = 0;
    double targetXStepInterval;
    double targetYStepInterval;
    double ratioX;
    double ratioY;

    int64_t currentX_printed=-1;
    int64_t currentY_printed=-1;
    bool readyState=true;
    bool printed_error=false;

    // Fractional Step Timing Correction (Fixed-Point Error Accumulation)
    void alignmentInterval(absolute_time_t now);

    double xIntervalError;
    double yIntervalError;
    absolute_time_t lastAlignmentTime;
    uint32_t alignmentIntervalTime;
    uint32_t alignmentCount;
    uint32_t alignmentCountX;
    uint32_t alignmentCountY;
    double alignmentIntervalFactor;
    double correctionFactor; 
    int32_t maxAllowedDeviation;
    
};

#endif // MOTORPLANNER_H