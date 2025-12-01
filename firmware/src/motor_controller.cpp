// motor_controller.cpp
#include "motor_controller.h"

#include "bilog.h"


StepperMotorController::StepperMotorController(CommandRingBuffer &queue_): flipSpeedStateX(), flopSpeedStateX(), flipSpeedStateY(), flopSpeedStateY(),
                                                                           flipSpeedStateA(), flopSpeedStateA(), flipSpeedStateB(), flopSpeedStateB(),
                                                                           flipStepStateX(),  flopStepStateX(),  flipStepStateY(),  flopStepStateY(),
                                                                           flipStepStateA(),  flopStepStateA(),  flipStepStateB(),  flopStepStateB(),
                                                                           activeSpeedStateX(&flipSpeedStateX), waitingSpeedStateX(&flopSpeedStateX),
                                                                           activeSpeedStateY(&flipSpeedStateY), waitingSpeedStateY(&flopSpeedStateY),
                                                                           activeSpeedStateA(&flipSpeedStateA), waitingSpeedStateA(&flopSpeedStateA),
                                                                           activeSpeedStateB(&flipSpeedStateB), waitingSpeedStateB(&flopSpeedStateB),
                                                                           activeSpeedStates{&activeSpeedStateX, &activeSpeedStateY,
                                                                               &activeSpeedStateA, &activeSpeedStateB},
                                                                           waitingSpeedStates{&waitingSpeedStateX, &waitingSpeedStateY,
                                                                               &waitingSpeedStateA, &waitingSpeedStateB},
                                                                           activeStepStateX(&flipStepStateX), waitingStepStateX(&flopStepStateX),
                                                                           activeStepStateY(&flipStepStateY), waitingStepStateY(&flopStepStateY),
                                                                           activeStepStateA(&flipStepStateA), waitingStepStateA(&flopStepStateA),
                                                                           activeStepStateB(&flipStepStateB), waitingStepStateB(&flopStepStateB),
                                                                           activeStepStates{&activeStepStateX, &activeStepStateY,
                                                                               &activeStepStateA, &activeStepStateB},
                                                                           waitingStepStates{&waitingStepStateX, &waitingStepStateY,
                                                                               &waitingStepStateA, &waitingStepStateB},
                                                                           currentPositionX(0),currentPositionY(0),queue(queue_) {
    flipStepStateX.positionMode = true;
    flipStepStateY.positionMode = true;
    flopStepStateX.positionMode = true;
    flopStepStateY.positionMode = true;
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

bool StepperMotorController::addToBuffer(uint8_t ctrlFlags,
                                         uint32_t stepsX, uint32_t stepsY,
                                         uint32_t intSpeedPartX, uint32_t intSpeedPartY,
                                         uint32_t errorIncrementX, uint32_t errorIncrementY) {
    return queue.push(ctrlFlags, stepsX, stepsY, intSpeedPartX, intSpeedPartY, errorIncrementX, errorIncrementY);
}


void StepperMotorController::setWaitingSpeed(const AXIS axis, const uint32_t stepInterval, const uint32_t errorIncrement) {
    MotorSpeedState & speedState = **waitingSpeedStates[axis];
    speedState.stepInterval = stepInterval;
    speedState.errorIncrement = errorIncrement;
    speedState.errorAccumulator = 0;
}

void StepperMotorController::setWaitingSteps(const AXIS axis, const  uint32_t steps, const bool direction, const bool positionMode) {
    MotorStepState & stepState = **waitingStepStates[axis];
    stepState.stepsRemaining = steps;
    stepState.direction = direction;
    stepState.positionMode = positionMode;
}


void StepperMotorController::flipFlopSpeedState(const AXIS axis) {
    MotorSpeedState* temp = *activeSpeedStates[axis];
    *activeSpeedStates[axis] = *waitingSpeedStates[axis];
    *waitingSpeedStates[axis] = temp;
}

void StepperMotorController::flipFlopStepState(const AXIS axis) {
    MotorStepState* temp = *activeStepStates[axis];
    *activeStepStates[axis] = *waitingStepStates[axis];
    *waitingStepStates[axis] = temp;
}


bool needStep(const absolute_time_t now) { // каждую микросекунду
    if (absolute_time_diff_us(delayed_by_us(stepState.stepLastTime, speedState.stepInterval), now)<0) return false; // не настало время шага
    uint32_t prev = speedState.errorAccumulator; // запоминаем текущее значение аккумулятора ощибки
    speedState.errorAccumulator += speedState.errorIncrement; // добавляем дробную часть
    if (speedState.errorAccumulator < prev) { // если было переполнение
        stepState.stepLastTime = delayed_by_us(now, 1); // то следущий шаг выполним на 1 микросекунду позже
    } else {
        stepState.stepLastTime = now;
    }
    return true; // сделать время шаг настало


bool StepperMotorController::needStep(const AXIS axis, const absolute_time_t now) {
    MotorStepState& stepState = **activeStepStates[axis];
    MotorSpeedState& speedState = **activeSpeedStates[axis];
    readyMask |= (1<<axis);
    if (stepState.positionMode && stepState.stepsRemaining != 0) readyMask &= ~(1<<axis);
    if (speedState.stepInterval == 0) return false;
    if (absolute_time_diff_us(delayed_by_us(stepState.stepLastTime, speedState.stepInterval), now)<0) return false;

    uint32_t prev = speedState.errorAccumulator;
    speedState.errorAccumulator += speedState.errorIncrement;
    if (speedState.errorAccumulator < prev) {
        stepState.stepLastTime = delayed_by_us(now, 1);
    } else {
        stepState.stepLastTime = now;
    }
    if (!stepState.positionMode) return true;
    if (stepState.stepsRemaining == 0) return false;
    stepState.stepsRemaining--;
    currentPositionX += axis == AXIS_X && stepState.direction ? 1 : axis == AXIS_X && !stepState.direction ? -1 : 0;
    currentPositionY += axis == AXIS_Y && stepState.direction ? 1 : axis == AXIS_Y && !stepState.direction ? -1 : 0;
    return true;
}



void StepperMotorController::adjustDirections() {
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        const MotorStepState & stepState = **activeStepStates[axis];
        if (lastDirection[axis]!=stepState.direction) {
            lastDirection[axis]=stepState.direction;
            gpio_put(AXIS_DIRECTION_PIN[axis], AXIS_DIRECTION_INVERT[axis] ^ stepState.direction);
        }
    }
}

void StepperMotorController::doSteps(const uint8_t doStepMask){
    const uint8_t lBits = (MOTOR_STEP_INVERT ? ~doStepMask : doStepMask) & 0x0F;
    const uint8_t stepsBits = STEPS_TEMPLATE | lBits;
    pio_sm_put_blocking(pio0, 0, stepsBits);
}

bool StepperMotorController::popXYStates() {
    if ((readyMask & READY_XY) != READY_XY) return false;
    MotorCommand cmd{0};
    if (queue.pop(cmd)) {
        setWaitingSteps(AXIS_X, cmd.stepsX, cmd.ctrlFlags & 1, true);
        setWaitingSteps(AXIS_Y, cmd.stepsY, (cmd.ctrlFlags >> 1) & 1, true);
        setWaitingSpeed(AXIS_X, cmd.stepIntervalX, cmd.errorIncrementX);
        setWaitingSpeed(AXIS_Y, cmd.stepIntervalY, cmd.errorIncrementY);
        flipFlopSpeedState(AXIS_X);
        flipFlopSpeedState(AXIS_Y);
        flipFlopStepState(AXIS_X);
        flipFlopStepState(AXIS_Y);
        return true;
    }
    return false;
}

bool StepperMotorController::popAStates() {
    if ((readyMask & READY_A) != READY_A) return false;
    return true;
}

bool StepperMotorController::popBStates() {
    if ((readyMask & READY_B) != READY_B) return false;
    return true;
}

void StepperMotorController::tick(const absolute_time_t now) {
    uint8_t needStepBits = 0;
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        needStepBits |= (needStep(static_cast<AXIS>(axis), now) & 1)<<axis;
    }
    if (needStepBits!=0) doSteps(needStepBits);

    bool needAdjustDirections = popXYStates();
    needAdjustDirections |= popAStates();
    needAdjustDirections |= popBStates();
    if (needAdjustDirections) adjustDirections();
}
