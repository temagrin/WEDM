// motor_controller.cpp
#include "motor_controller.h"

StepperMotorController::StepperMotorController() :
        flipStepState{0, 0, 0, 0, 0, true},
        flopStepState{0, 0, 0, 0, 0, true},
        flipSpeedStateXY(), flopSpeedStateXY(),
        flipSpeedStateA(), flopSpeedStateA(), flipSpeedStateB(), flopSpeedStateB(),
        flipDirectionStateX(), flopDirectionStateX(), flipDirectionStateY(), flopDirectionStateY(),
        flipDirectionStateA(), flopDirectionStateA(), flipDirectionStateB(), flopDirectionStateB(),
        activeStepState(&flipStepState), waitingStepState(&flopStepState),
        activeSpeedStates{&flipSpeedStateXY, &flipSpeedStateA, &flipSpeedStateB},
        waitingSpeedStates{&flopSpeedStateXY, &flopSpeedStateA, &flopSpeedStateB},
        activeDirectionStates{&flipDirectionStateX, &flipDirectionStateY,
                              &flipDirectionStateA, &flipDirectionStateB},
        waitingDirectionStates{&flopDirectionStateX, &flopDirectionStateY,
                               &flopDirectionStateA,&flopDirectionStateB} {}

void StepperMotorController::initPin(const uint16_t _pin, const bool defaultValue) {
    gpio_init(_pin);
    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, defaultValue);
}

void StepperMotorController::initMotors() {
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
    setPowerXY(ctrlFlags & 0x1);         // flag1 - питание моторов XY
    setPowerA((ctrlFlags >> 1) & 0x1);   // flag2 - питание мотора A
    setPowerB((ctrlFlags >> 2) & 0x1);   // flag3 - питание мотора B
    return true;
}

bool StepperMotorController::addToBuffer(const uint8_t ctrlFlags, const uint32_t stepsX, const uint32_t stepsY,
                                         const uint32_t intSpeedPart, const uint32_t errorIncrement) {
    return queue.push(ctrlFlags, stepsX, stepsY, intSpeedPart, errorIncrement);
}

void StepperMotorController::setIntervalA(const bool direction, const uint32_t intSpeedPart, const uint32_t errorIncrement) {
    setWaitingSpeed(G_AXIS_A, intSpeedPart, errorIncrement);
    setWaitingDirection(AXIS_A, direction);
    flipFlopSpeedState(G_AXIS_A);
    flipFlopDirectionState(AXIS_A);
}

void StepperMotorController::setIntervalB(const bool direction, const uint32_t intSpeedPart, const uint32_t errorIncrement) {
    setWaitingSpeed(G_AXIS_B, intSpeedPart, errorIncrement);
    setWaitingDirection(AXIS_B, direction);
    flipFlopSpeedState(G_AXIS_B);
    flipFlopDirectionState(AXIS_B);
}


void StepperMotorController::setBreakFactor(uint8_t value) {
    if (breakFactor != value){
        breakFactor = value;
    }
}

void StepperMotorController::setWaitingSpeed(const G_AXIS g_axis, const uint32_t stepInterval,
                                             const uint32_t errorIncrement) {
    MotorSpeedState &speedState = *waitingSpeedStates[g_axis];
    speedState.stepLastTime = get_absolute_time();
    speedState.stepInterval = stepInterval;
    speedState.errorIncrement = errorIncrement;
    speedState.errorAccumulator = 0;
}

void StepperMotorController::setWaitingSteps(const uint32_t stepsX, const uint32_t stepsY) {
    MotorBresenhamState &stepState = *waitingStepState;
    stepState.targetX = (int32_t) stepsX;
    stepState.targetY = (int32_t) stepsY;
    stepState.taskCurrentX = 0;
    stepState.taskCurrentY = 0;
    stepState.errorPosition = stepState.targetX - stepState.targetY;
    stepState.finished = false;
}

void StepperMotorController::setWaitingDirection(AXIS axis, bool direction) {
    MotorDirectionState &directionState = *waitingDirectionStates[axis];
    directionState.direction = direction;
}


void StepperMotorController::flipFlopSpeedState(const G_AXIS axis) {
    MotorSpeedState &temp = *activeSpeedStates[axis];
    *activeSpeedStates[axis] = *waitingSpeedStates[axis];
    *waitingSpeedStates[axis] = temp;
}


void StepperMotorController::flipFlopStepState() {
    MotorBresenhamState &temp = *activeStepState;
    *activeStepState = *waitingStepState;
    *waitingStepState = temp;
}

void StepperMotorController::flipFlopDirectionState(AXIS axis) {
    MotorDirectionState &oldDirectionState = *activeDirectionStates[axis];
    MotorDirectionState &newDirectionState = *waitingDirectionStates[axis];
    *activeDirectionStates[axis] = newDirectionState;
    *waitingDirectionStates[axis] = oldDirectionState;
    if (newDirectionState.direction != oldDirectionState.direction) {
        gpio_put(AXIS_DIRECTION_PIN[axis], AXIS_DIRECTION_INVERT[axis] ^ newDirectionState.direction);
    }
}

uint8_t StepperMotorController::needBresenhamSteps() {
    MotorBresenhamState &stepState = *activeStepState;
    int stepMask = 0xC;
    if (stepState.finished) return stepMask;
    if (stepState.taskCurrentX == stepState.targetX
        && stepState.taskCurrentY == stepState.targetY) {
        stepState.finished = true;
        return stepMask;
    }
    int error2 = stepState.errorPosition * 2;
    if (error2 > -stepState.targetY) {
        stepState.errorPosition -= stepState.targetY;
        stepState.taskCurrentX += 1;
        stepMask |= 0x01;  // ставим xxx1  - разрешить шаг X
    }
    if (error2 < stepState.targetX) {
        stepState.errorPosition += stepState.targetX;
        stepState.taskCurrentY += 1;
        stepMask |= 0x02; // ставим xx1x - разрешить шаг Y
    }
    return stepMask;
}

bool StepperMotorController::needStep(const G_AXIS axis, const absolute_time_t now) {
    MotorSpeedState &speedState = *activeSpeedStates[axis];
    if (speedState.stepInterval == 0) return false;
    if (absolute_time_diff_us(delayed_by_us(speedState.stepLastTime, speedState.stepInterval), now) < 0) return false;

    uint32_t prev = speedState.errorAccumulator;
    speedState.errorAccumulator += speedState.errorIncrement;
    if (speedState.errorAccumulator < prev) {
        speedState.stepLastTime = delayed_by_us(now, 1);
    } else {
        speedState.stepLastTime = now;
    }
    return true;
}


void StepperMotorController::doSteps(const uint8_t doStepMask) {
    const uint8_t lBits = (MOTOR_STEP_INVERT ? ~doStepMask : doStepMask) & 0x0F;
    const uint8_t stepsBits = STEPS_TEMPLATE | lBits;
    pio_sm_put_blocking(pio0, 0, stepsBits);
}

void StepperMotorController::popXYStates() {
    MotorBresenhamState &stepState = *activeStepState;
    if (!stepState.finished) return;
    MotorCommand cmd{0};
    if (queue.pop(cmd)) {
        setWaitingSteps(cmd.stepsX, cmd.stepsY);
        setWaitingDirection(AXIS_X, (cmd.ctrlFlags & 1));
        setWaitingDirection(AXIS_Y, (cmd.ctrlFlags >> 1 & 1));
        setWaitingSpeed(G_AXIS_XY, cmd.stepInterval, cmd.errorIncrement);
        flipFlopSpeedState(G_AXIS_XY);
        flipFlopDirectionState(AXIS_X);
        flipFlopDirectionState(AXIS_Y);
        flipFlopStepState();
    }
}

bool StepperMotorController::breakStepXY() {
    if (breakFactor == 0)  return false;   // не тормозим
    if (breakFactor >= 63)  return true;   // полная остановка
    stepSkipCounter += breakFactor;
    if (stepSkipCounter >= 63) {
        stepSkipCounter -= 63;
        return true;
    }
    return false;
}

void StepperMotorController::applyStepsToCurrentPosition(uint8_t stepsMask) {
    if (stepsMask & 0x1) currentPositionX += activeDirectionStates[AXIS_X]->direction ? 1 : -1;
    if (stepsMask >> 1 & 0x1) currentPositionY += activeDirectionStates[AXIS_Y]->direction ? 1 : -1;
}

void StepperMotorController::tick(const absolute_time_t now) {
    uint8_t needStepBits = 0;
    for (uint8_t axis = 0; axis < G_AXIS_COUNT; axis++) {
        needStepBits |= (needStep(static_cast<G_AXIS>(axis), now) & 1) << (axis + 1);
    }  // тут битова маска такая [G_AXIS_B G_AXIS_A G_AXIS_XY 0]
    if (breakStepXY()) needStepBits &= 0xC;
    if ((needStepBits & 0x02) == 0x02) { // если бит G_AXIS_XY стоит
        needStepBits |= 0x01; // то сделаем [AXIS_B AXIS_A AXIS_Y AXIS_X] xx11
        needStepBits &= needBresenhamSteps(); // и применим шагание по брезенхаму ( первые два младших бита это как раз AXIS_X и AXIS_Y
        applyStepsToCurrentPosition(needStepBits);
    }
    if (needStepBits != 0) doSteps(needStepBits);
    popXYStates(); // проверим если XY дошагали, может в буфере есть новое задание
}
