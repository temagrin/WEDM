// motor_planner.cpp
#include "motor_planner.h"


StepperMotorPlanner::StepperMotorPlanner(StepperMotor* motorX, StepperMotor* motorY)
    : motorX(motorX), motorY(motorY){}


bool StepperMotorPlanner::moveTo(int64_t newX, int64_t newY, uint32_t tool_speed) { // постановка задачи
    if (!readyState) return false; // вернем неуспех потому что заняты еще.
    readyState=false; // становимся занятым для других задачь
    int64_t dx = newX - currentX; 
    int64_t dy = newY - currentY;
    bool dirX = dx>=0;
    bool dirY = dy>=0;
    uint32_t adx = abs(dx);
    uint32_t ady = abs(dy);
    
    double dist = hypot((double)adx, (double)ady);
    ratioX = (double)adx/dist;
    ratioY = (double)ady/dist;
    
    targetXStepInterval = minAllowedStepInterval;
    targetYStepInterval = minAllowedStepInterval;

    // если движение по одной оси, ничего страшного что будет какой то минимальный интервал. 
    // у класса мотор-контроллера есть проверка на наличие шагов оставшихся шагов, когда там ничего - он не будет шагать
    if (ratioX!=0) targetXStepInterval = (double)1e6/(tool_speed*ratioX);
    if (ratioY!=0) targetYStepInterval = (double)1e6/(tool_speed*ratioY);

    targetXStepInterval = constrain(targetXStepInterval, minAllowedStepInterval, maxAllowedStepInterval);
    targetYStepInterval = constrain(targetYStepInterval, minAllowedStepInterval, maxAllowedStepInterval);
    
    xStepInterval = (uint32_t) targetXStepInterval;
    yStepInterval = (uint32_t) targetYStepInterval;

    Serial.print("dx=");Serial.print(dx);Serial.print("\n");
    Serial.print("dy=");Serial.print(dy);Serial.print("\n");
    Serial.print("tool_speed=");Serial.print(tool_speed);Serial.print("\n");
    Serial.print("distXY=");Serial.print(dist);Serial.print("\n");
    Serial.print("ratioX=");Serial.print(ratioX);Serial.print("\n");
    Serial.print("ratioY=");Serial.print(ratioY);Serial.print("\n");

    Serial.print("targetXStepInterval=");Serial.print(targetXStepInterval);Serial.print("\n");
    Serial.print("targetYStepInterval=");Serial.print(targetYStepInterval);Serial.print("\n");
    Serial.print("startIntXStepInterval=");Serial.print(xStepInterval);Serial.print("\n");
    Serial.print("startIntYStepInterval=");Serial.print(yStepInterval);Serial.print("\n");


    // предрасчет параметров для Fractional Step Timing Correction
    xIntervalError = 0.0;
    yIntervalError = 0.0;
    
    uint32_t baseInterval = xStepInterval>yStepInterval? xStepInterval: yStepInterval;
    double maxTargetInterval = std::max(targetXStepInterval, targetYStepInterval);
    alignmentIntervalFactor = floor(2.0 * maxAccumulatedError / maxTargetInterval);

    alignmentIntervalTime = std::min(2.0 * deltaT * baseInterval,
                            std::max(minAllowedAlignmentInterval, baseInterval * 10.0));

    correctionFactor = deltaT * baseInterval / alignmentIntervalTime;
    maxAllowedDeviation = std::max( (int)(alignmentIntervalFactor * 0.5), 1 );
    

    Serial.println("----------");
    
    Serial.print("correctionFactor=");Serial.print(correctionFactor);Serial.print("\n");
    Serial.print("maxAllowedDeviation=");Serial.print(maxAllowedDeviation);Serial.print("\n");
    Serial.print("baseInterval=");Serial.print(baseInterval);Serial.print("\n");
    Serial.print("maxTargetInterval=");Serial.print(maxTargetInterval);Serial.print("\n");
    Serial.print("alignmentIntervalFactor=");Serial.print(alignmentIntervalFactor);Serial.print("\n");
    Serial.print("alignmentIntervalTime=");Serial.print(alignmentIntervalTime);Serial.print("\n");
    
    
    motorX->setStepInterval(xStepInterval);
    motorX->setDirection(dx>=0);
    motorX->setStepsRemaining(adx);
    motorY->setStepInterval(yStepInterval);
    motorY->setDirection(dy>=0);
    motorY->setStepsRemaining(ady);
    motorX->run();
    motorY->run();
    Serial.println("----------");
    return true; //задачу поставили
}


void StepperMotorPlanner::alignmentInterval(absolute_time_t now){
    lastAlignmentTime = now;
    alignmentCount++;

    const MotorState& stateX = motorX->getMotorState();
    const MotorState& stateY = motorY->getMotorState();
    
    if ((stateX.stepsRemaining + stateY.stepsRemaining)==0) return;
    int8_t intervalModX = 0;
    int8_t intervalModY = 0;
    
    double progressX = (double)stateX.stepsDone/(stateX.stepsRemaining + stateX.stepsDone);
    double progressY = (double)stateY.stepsDone/(stateY.stepsRemaining + stateY.stepsDone);
    double progressTotal = (double)(stateX.stepsDone+stateY.stepsDone)/(stateX.stepsRemaining + stateX.stepsDone + stateY.stepsRemaining + stateY.stepsDone);
    
    // Коррекция интервала X с накоплением ошибки
    double targetCorrectionX = 0.0;
    if (progressX > progressTotal) {
        targetCorrectionX = correctionFactor;    // увеличить интервал (замедлить)
    } else if (progressX < progressTotal) {
        targetCorrectionX = -correctionFactor;   // уменьшить интервал (ускорить)
    }

    xIntervalError += targetCorrectionX;

    if (xIntervalError >= 1.0) {
        if (xStepInterval < targetXStepInterval+maxAllowedDeviation) {
            intervalModX = 1;
            xIntervalError -= 1.0;
        } else {
            xIntervalError = 1.0; // ограничение по maxInterval
        }
    } else if (xIntervalError <= -1.0) {
        if (xStepInterval > targetXStepInterval-maxAllowedDeviation) {
            intervalModX = -1;
            xIntervalError += 1.0;
        } else {
            xIntervalError = -1.0; // ограничение по minInterval
        }
    }

    // Коррекция интервала X с накоплением ошибки
    double targetCorrectionY = 0.0;
    if (progressY > progressTotal) {
        targetCorrectionY = correctionFactor;
    } else if (progressY < progressTotal) {
        targetCorrectionY = -correctionFactor;
    }

    yIntervalError += targetCorrectionY;

    if (yIntervalError >= 1.0) {
        if (yStepInterval < targetYStepInterval+maxAllowedDeviation) {
            intervalModY = 1;
            yIntervalError -= 1.0;
        } else {
            yIntervalError = 1.0;
        }
    } else if (yIntervalError <= -1.0) {
        if (yStepInterval > targetYStepInterval-maxAllowedDeviation) {
            intervalModY = -1;
            yIntervalError += 1.0;
        } else {
            yIntervalError = -1.0;
        }
    }

    if (intervalModX!=0) {
        xStepInterval+=intervalModX;
        alignmentCountX++;
        motorX->setStepInterval(xStepInterval);
    }

    if (intervalModY!=0) {
        yStepInterval+=intervalModY;
        alignmentCountY++;
        motorY->setStepInterval(yStepInterval);
    }
    
}

void StepperMotorPlanner::reset(){
    motorX->resetAbsoluteSteps();
    motorY->resetAbsoluteSteps();
    alignmentCount = 0;
    alignmentCountX = 0;
    alignmentCountY = 0;
    currentX=0;
    currentY=0;
    printed_error=false;
}

bool StepperMotorPlanner::ready(){
    return readyState;
}

void StepperMotorPlanner::handlePlaner(){ // вызывается часто
    absolute_time_t now = get_absolute_time();
    absolute_time_t elastedTime = delayed_by_us(lastAlignmentTime, alignmentIntervalTime);
    if (absolute_time_diff_us(elastedTime, now) > 0) alignmentInterval(now);

    const MotorState& stateX = motorX->getMotorState();
    const MotorState& stateY = motorY->getMotorState();
    if ((stateX.stepsRemaining+stateY.stepsRemaining == 0) && !printed_error){
        printed_error = true;
        Serial.println("Results after move:");
        Serial.print("Total time: ~");Serial.print(alignmentCount * alignmentIntervalTime);Serial.print("\n");
        Serial.print("alignmentIntervalTime=");Serial.print(alignmentIntervalTime);Serial.print("\n");
        Serial.print("alignmentIntervalFactor=");Serial.print(alignmentIntervalFactor);Serial.print("\n");
        Serial.print("lastXStepInterval=");Serial.print(stateX.stepInterval);Serial.print("\n");
        Serial.print("lastYStepInterval=");Serial.print(stateY.stepInterval);Serial.print("\n");
        Serial.print("lastXStepTime=");Serial.print(stateX.stepLastTime);Serial.print("\n");
        Serial.print("lastYStepTime=");Serial.print(stateY.stepLastTime);Serial.print("\n");
        Serial.print("alignmentCount=");Serial.print(alignmentCount);Serial.print("\n");
        Serial.print("alignmentCountX=");Serial.print(alignmentCountX);Serial.print("\n");
        Serial.print("alignmentCountY=");Serial.print(alignmentCountY);Serial.print("\n");
        Serial.print("Error stop motor times=");Serial.print(absolute_time_diff_us(stateY.stepLastTime, stateX.stepLastTime));Serial.print("\n");
        Serial.println("----------");
        readyState=true;
    }
}


    // currentX = stateX.absoluteSteps;
    // currentY = stateY.absoluteSteps;
    // if (currentX_printed!=currentX || currentY_printed != currentY){
    //     Serial.print("New current position: x=");Serial.print(currentX);Serial.print(" y=");Serial.print(currentY);Serial.print("\n");
    //     currentX_printed = currentX;
    //     currentY_printed = currentY;
    // }

