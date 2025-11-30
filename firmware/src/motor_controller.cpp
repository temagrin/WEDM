// motor_controller.cpp
#include "motor_controller.h"


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

/////////////////////////////////   МЕДЛЕННЫЙ ДОМЕН   ///////////////////////////////////////

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

bool StepperMotorController::commonControl(const uint8_t ctrlFlags) {
    setPowerXY(ctrlFlags & 0x1);       // flag1 - питание моторов XY
    setPowerA((ctrlFlags>>1) & 0x1);   // flag2 - питание мотора A
    setPowerB((ctrlFlags>>2) & 0x1);   // flag3 - питание мотора B
    if (ctrlFlags>>3 & 0x1) {          // flag4 - сбросить машинные координаты
        currentPositionX = 0;
        currentPositionY = 0;
    }
    return true;
}



void StepperMotorController::setWaitingSpeedCalculateXY(const uint32_t speedX, const uint32_t speedY, const bool immediately) {
    setWaitingSpeedCalculate(AXIS_X, speedX);
    setWaitingSpeedCalculate(AXIS_Y, speedY);
    if (immediately) {
        uint16_t commandFIFO = 0;
        commandFIFO |= makeFIFOCommand(AXIS_X, SPEED, IMMEDIATE);
        commandFIFO |= makeFIFOCommand(AXIS_Y, SPEED, IMMEDIATE);
        applyNewStates(commandFIFO);
    }
}

void StepperMotorController::setWaitingSpeedCalculate(const AXIS axis, const uint32_t speed) {
    if (speed==0) {setWaitingSpeed(axis, 0, 0); return;}
    const double stepInterval = 1e6/speed;
    const auto intSpeedPart = static_cast<uint32_t>(stepInterval);
    const double fracPart = stepInterval - static_cast<double>(intSpeedPart);
    const auto errorIncrement = static_cast<uint32_t>(fracPart * DOUBLE_SCALE);
    setWaitingSpeed(axis, intSpeedPart, errorIncrement);
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


uint16_t StepperMotorController::makeFIFOCommand(const AXIS axis, const CommandType commandType, const CommandTime commandTime) {
    return 1 << (axis + commandTime + commandType);
}

/**
 * обертку выполнять строго на другом ядре относительно быстрого домена.
 * если doFlipFlop на core1 то эту функцию вызывать на core0
 * @param updateMask перевернуть waiting и active states.
 * формат маски описан в StepperMotorController::doFlipFlop
 * также написал хелпер для сбора команды - StepperMotorController::makeFIFOCommand
 */
void StepperMotorController::applyNewStates(const uint16_t updateMask) {
    sentReadyFlipFlop = false; // что то отправили, теперь ждем подтверждения выполнения
    multicore_fifo_push_blocking(updateMask);
}


void StepperMotorController::checkBuffer() {
    if (multicore_fifo_rvalid()) {
        if (multicore_fifo_pop_blocking()!=0) {
            readyFlipFlop = true; // фиксируемся что последняя команда из фифо выполнилась, можно читать кольцевой буфер
        }
    }
    if (!readyFlipFlop) return;
    MotorCommand cmd{0};
    if (queue.pop(cmd)) {
        setWaitingSteps(AXIS_X, cmd.stepsX, cmd.speedX>0, true);
        setWaitingSteps(AXIS_Y, cmd.stepsY, cmd.speedY>0, true);
        setWaitingSpeedCalculateXY(abs(cmd.speedX), abs(cmd.speedY), false);

        uint16_t commandFIFO = 0;
        commandFIFO |= makeFIFOCommand(AXIS_X, SPEED, BY_READY);
        commandFIFO |= makeFIFOCommand(AXIS_X, STEP, BY_READY);
        commandFIFO |= makeFIFOCommand(AXIS_Y, SPEED, BY_READY);
        commandFIFO |= makeFIFOCommand(AXIS_Y, STEP, BY_READY);

        applyNewStates(commandFIFO);
    }
}


/////////////////////////////////   БЫСТРЫЙ ДОМЕН   ///////////////////////////////////////
static bool led1Value = false;
static bool led2Value = false;

void StepperMotorController::flipFlopSpeedState(const AXIS axis) {
    gpio_put(20, led2Value ^= 1);

    MotorSpeedState* temp = *activeSpeedStates[axis];
    *activeSpeedStates[axis] = *waitingSpeedStates[axis];
    *waitingSpeedStates[axis] = temp;
}

void StepperMotorController::flipFlopStepState(const AXIS axis) {
    gpio_put(22, led1Value ^= 1);

    MotorStepState* temp = *activeStepStates[axis];
    *activeStepStates[axis] = *waitingStepStates[axis];
    *waitingStepStates[axis] = temp;
}


bool StepperMotorController::needStep(const AXIS axis, const absolute_time_t now) {
    MotorStepState& stepState = **activeStepStates[axis];
    MotorSpeedState& speedState = **activeSpeedStates[axis];
    if (speedState.stepInterval == 0) return false; // шагание выключено
    if (absolute_time_diff_us(delayed_by_us(stepState.stepLastTime, speedState.stepInterval), now)<0) return false;
    speedState.errorAccumulator+=speedState.errorIncrement;
    if (speedState.errorAccumulator>SCALE){
        stepState.stepLastTime = delayed_by_us(now, 1);
        speedState.errorAccumulator-=SCALE;
    } else {
        stepState.stepLastTime = now;
    }
    readyMask |= (1<<axis);
    if (!stepState.positionMode) return true;
    if (stepState.stepsRemaining == 0) return false;
    readyMask &= ~(1<<axis);
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


/**
 * Метод производящий переворот нужных стейтов согласно набранной команде commandMask
 * формат команды commandMask
 * commandMask[0] - бит немедленного переворота стейта скорости оси X
 * commandMask[1] - бит немедленного переворота стейта скорости оси Y
 * commandMask[2] - бит немедленного переворота стейта скорости оси A
 * commandMask[3] - бит немедленного переворота стейта скорости оси B
 * commandMask[4] - бит немедленного переворота стейта шагов оси X
 * commandMask[5] - бит немедленного переворота стейта шагов оси Y
 * commandMask[6] - бит немедленного переворота стейта шагов оси A
 * commandMask[7] - бит немедленного переворота стейта шагов оси B
 * commandMask[8] - бит переворота по готовности стейта скорости оси X
 * commandMask[9] - бит переворота по готовности стейта скорости оси Y
 * commandMask[10] - бит переворота по готовности стейта скорости оси A
 * commandMask[11] - бит переворота по готовности стейта скорости оси B
 * commandMask[12] - бит переворота по готовности стейта шагов оси X
 * commandMask[13] - бит переворота по готовности стейта шагов оси Y
 * commandMask[14] - бит переворота по готовности стейта шагов оси A
 * commandMask[15] - бит переворота по готовности стейта шагов оси B
 * @return если был хотя бы один из переворотов - вернем true
 */

bool StepperMotorController::doFlipFlop() {
    if (commandMask == 0) return false; // еще не поступали команды
    readyMask &= 0x0F; // очистим старшие 4ре бита так как могли остаться с прошлого раза
    if ((readyMask & 0x03) != 0x03) readyMask &= ~0x03; // Обнуляем готовность X и Y если один из них не готов
    readyMask |= (readyMask << 4); // актуализируем старшие 4ре бита из младших
    uint8_t doItMask = commandMask & 0xFF; // берем немедленную часть
    doItMask |= ((commandMask >> 8) & 0xFF) & readyMask; // накладываем часть по готовности
    if (doItMask == 0) return false; // ничего не меняем - не готовы

    // Применяем маски
    const uint8_t speedFlipMask = doItMask & 0x0F; // младшие 4ре это флип скорости по осям
    const uint8_t stepFlipMask = (doItMask >> 4) & 0x0F; // старшие 4ре это флип шагов по осям
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        if (speedFlipMask & (1 << axis)) {
            flipFlopSpeedState(static_cast<AXIS>(axis));
        }
        if (stepFlipMask & (1 << axis)) {
            flipFlopStepState(static_cast<AXIS>(axis));
        }
    }
    // сбрасываем в команде выполненные биты
    commandMask &= ~((doItMask & 0xFF) | ((doItMask & 0xFF) << 8));
    readyMask &= ~(doItMask & 0xFF);
    return true; // говорим коду дальше что нужно дополнительно проверить направления вращения
}


void StepperMotorController::tick(const absolute_time_t now) {
    uint8_t needStepBits = 0;
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        needStepBits |= (needStep(static_cast<AXIS>(axis), now) & 1)<<axis; // проверяем каждую ось нужно ли сделать по ней шаг?
    }
    if (needStepBits!=0) doSteps(needStepBits); // делаем шаги по нужным осям если пора их делать
    if (multicore_fifo_rvalid()) commandMask |= multicore_fifo_pop_blocking();
    if (doFlipFlop()) adjustDirections(); // если был переворот, то проверим направления чтобы соответствовали
    if (commandMask==0 && !sentReadyFlipFlop) {
        sentReadyFlipFlop = true;
        multicore_fifo_push_blocking(1);
    }
}
