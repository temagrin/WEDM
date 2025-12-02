// motor_controller.cpp
#include "motor_controller.h"



StepperMotorController::StepperMotorController(){

}

void StepperMotorController::initPin(const uint16_t _pin, const bool defaultValue){
    gpio_init(_pin);
    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, defaultValue);
}

void StepperMotorController::initMotors(){
    const uint pio_offset = pio_add_program(pio0, &motor_step_sequence_program);
    motor_step_sequence_init(pio0, 0, pio_offset, STEP_BASE_PIN);

    initPin(DIR_X_PIN, MOTOR_X_DIR_INVERT);
    initPin(DIR_Y_PIN, MOTOR_Y_DIR_INVERT);
    initPin(DIR_A_PIN, MOTOR_A_DIR_INVERT);
    initPin(DIR_B_PIN, MOTOR_B_DIR_INVERT);

    initPin(EN_X_Y_PIN, MOTOR_ENABLE_INVERT);
    initPin(EN_A_PIN, MOTOR_ENABLE_INVERT);
    initPin(EN_B_PIN, MOTOR_ENABLE_INVERT);
    doSteps(0);
}

bool StepperMotorController::powerControl(const uint8_t ctrlFlags) {
    setPowerXY(ctrlFlags & 0x1);       // flag1 - питание моторов XY
    setPowerA((ctrlFlags>>1) & 0x1);   // flag2 - питание мотора A
    setPowerB((ctrlFlags>>2) & 0x1);   // flag3 - питание мотора B
    return true;
}

bool StepperMotorController::addToBuffer(const uint8_t ctrlFlags, const uint32_t stepsX, const uint32_t stepsY,
                                         const uint32_t intSpeedPart, const uint32_t errorIncrement) {
    return queue.push(ctrlFlags, stepsX, stepsY, intSpeedPart, errorIncrement);
}


void StepperMotorController::setWaitingSpeed(const G_AXIS g_axis, const uint32_t stepInterval, const uint32_t errorIncrement) {
    MotorSpeedState & speedState = **waitingSpeedStates[g_axis];
    speedState.stepLastTime = get_absolute_time();
    speedState.stepInterval = stepInterval;
    speedState.errorIncrement = errorIncrement;
    speedState.errorAccumulator = 0;
}

void StepperMotorController::setWaitingSteps(const uint32_t stepsX, const uint32_t stepsY) {
    MotorBresenhamState & stepState = *waitingStepState;
    stepState.targetX = (int32_t) stepsX;
    stepState.targetY = (int32_t) stepsY;
    stepState.taskCurrentX = 0;
    stepState.taskCurrentY = 0;
    stepState.errorPosition = stepState.targetX - stepState.targetY;
    stepState.finished = false;
}

void StepperMotorController::setWaitingDirection(AXIS axis, bool direction) {
    MotorDirectionState * directionState = *waitingDirectionStates[axis];
    directionState->direction = direction;
}


void StepperMotorController::flipFlopSpeedState(const G_AXIS axis) {
    MotorSpeedState* temp = *activeSpeedStates[axis];
    *activeSpeedStates[axis] = *waitingSpeedStates[axis];
    *waitingSpeedStates[axis] = temp;
}


void StepperMotorController::flipFlopStepState() {
    MotorBresenhamState& temp = *activeStepState;
    *activeStepState = *waitingStepState;
    *waitingStepState = temp;
}

void StepperMotorController::flipFlopDirectionState(AXIS axis) {
    MotorDirectionState* oldDirectionState = *activeDirectionStates[axis];
    MotorDirectionState* newDirectionState = *waitingDirectionStates[axis];
    *activeDirectionStates[axis] = newDirectionState;
    *waitingDirectionStates[axis] = oldDirectionState;
    if (newDirectionState->direction != oldDirectionState->direction){
        gpio_put(AXIS_DIRECTION_PIN[axis], AXIS_DIRECTION_INVERT[axis] ^ newDirectionState->direction);
    }
}


uint8_t StepperMotorController::needBresenhamSteps(){
    MotorBresenhamState& stepState = *activeStepState;
    int stepMask = 0xC; // ставим 1100 - чтобы потом побитово сделать И c маской по времени и оси A и B не трогать
    if (stepState.finished) return stepMask;
    if (stepState.taskCurrentX == stepState.targetX && stepState.taskCurrentY == stepState.targetY) {
        stepState.finished = true;
        return 0;
    }
    int error2 = stepState.errorPosition * 2;
    if (error2 > -stepState.targetY) {
        stepState.errorPosition -= stepState.targetY;
        stepState.taskCurrentX += 1;
        stepMask |= 0x01;  // ставим xxx1
    }
    if (error2 < stepState.targetX) {
        stepState.errorPosition += stepState.targetX;
        stepState.taskCurrentX += 1;
        stepMask |= 0x02; // ставим xx1x
    }
    return stepMask;
}

bool StepperMotorController::needStep(const G_AXIS axis, const absolute_time_t now) {
    MotorSpeedState& speedState = **activeSpeedStates[axis];
    if (speedState.stepInterval == 0) return false;
    if (absolute_time_diff_us(delayed_by_us(speedState.stepLastTime, speedState.stepInterval), now)<0) return false;

    uint32_t prev = speedState.errorAccumulator;
    speedState.errorAccumulator += speedState.errorIncrement;
    if (speedState.errorAccumulator < prev) {
        speedState.stepLastTime = delayed_by_us(now, 1);
    } else {
        speedState.stepLastTime = now;
    }
    return true;
}



void StepperMotorController::doSteps(const uint8_t doStepMask){
    const uint8_t lBits = (MOTOR_STEP_INVERT ? ~doStepMask : doStepMask) & 0x0F;
    const uint8_t stepsBits = STEPS_TEMPLATE | lBits;
    pio_sm_put_blocking(pio0, 0, stepsBits);
}

void StepperMotorController::popXYStates() {
    MotorBresenhamState& stepState = *activeStepState;
    if (!stepState.finished) return ;
    MotorCommand cmd{0};
    if (queue.pop(cmd)) {
        // заполняем waitingStates
        setWaitingSteps(cmd.stepsX, cmd.stepsY);
        setWaitingDirection(AXIS_X, (cmd.ctrlFlags & 1));
        setWaitingDirection(AXIS_Y, (cmd.ctrlFlags>>1 & 1));
        setWaitingSpeed(G_AXIS_XY, cmd.stepInterval, cmd.errorIncrement);
        // меняем active на заполненные waiting
        flipFlopSpeedState(G_AXIS_XY);
        flipFlopDirectionState(AXIS_X);
        flipFlopDirectionState(AXIS_Y);
        flipFlopStepState();
    }
}


void StepperMotorController::tick(const absolute_time_t now) {
    uint8_t needStepBits = 0;
    for (uint8_t axis = 1; axis <= G_AXIS_COUNT; axis++) {
        needStepBits |= (needStep(static_cast<G_AXIS>(axis), now) & 1)<<axis;
    }
    // 4 бита needStepBits - настало время: шага B, шага A, шага XY, бит 0 = 0

    if ((needStepBits & 0x02) == 0x02) { // интересует 2 бит - нужны ли шаги по XY
        needStepBits |= 0x01; // проставим бит 1 так как бит 2 стоит
        needStepBits &= needBresenhamSteps(); // применим маску шагов, помним что needBresenhamSteps() типа 0b11xx
        applyStepsToCurrentPosition(needStepBits);
    }

    if (needStepBits!=0) doSteps(needStepBits);

    popXYStates();

}

void StepperMotorController::applyStepsToCurrentPosition(uint8_t stepsMask) {
    if (stepsMask & 0x1) currentPositionX += activeDirectionStateX->direction ? 1 : -1;
    if (stepsMask>>1 & 0x1) currentPositionY += activeDirectionStateY->direction ? 1 : -1;
}




