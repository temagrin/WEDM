// motor_controller.cpp
#include "motor_controller.h"


StepperMotorController::StepperMotorController(){}

void StepperMotorController::initPin(uint16_t _pin, bool defaultValue){
    gpio_init(_pin);
    gpio_set_dir(_pin, GPIO_OUT);
    gpio_put(_pin, defaultValue);    
}



void StepperMotorController::initMotors(){
    // в блок PIO0 в стейт машину SM 0
    // uint pio_offset = pio_add_program(pio0, &motor_step_sequence_program);
    // motor_step_sequence_init(pio0, 0, pio_offset, STEP_BASE_PIN);
    
    initPin(DIR_X_PIN, MOTOR_X_DIR_INVERT);
    initPin(DIR_Y_PIN, MOTOR_Y_DIR_INVERT);
    initPin(DIR_A_PIN, MOTOR_A_DIR_INVERT);
    initPin(DIR_B_PIN, MOTOR_B_DIR_INVERT);
    
    initPin(EN_X_Y_PIN, MOTOR_ENABLE_INVERT);
    initPin(EN_A_PIN, MOTOR_ENABLE_INVERT);
    initPin(EN_B_PIN, MOTOR_ENABLE_INVERT);
        
    // на всякий случай "обнуляем" время последнего шага
    doSteps(0); // установим дефолтное значение на steps пинах
    absolute_time_t now = get_absolute_time();
    stateA.stepLastTime = now;
    stateB.stepLastTime = now;
    stateX.stepLastTime = now;
    stateY.stepLastTime = now;
}


// вызывается каждую микросекунду
bool StepperMotorController::needStep(MotorState* state, absolute_time_t now) {
        if (state->stepInterval == 0) return false; // шагание выключено
        if (absolute_time_diff_us(delayed_by_us(state->stepLastTime, state->stepInterval), now)<0) return false; // время не пришло
        state->errorAccumulator+=state->errorIncrement; // добавляем в ошибку величину интервала которую мы не дождались
        // если накопилось неучтенного задержки, то следуйщий шаг надо сделать на квант времени позже
        if (state->errorAccumulator>SCALE){
            state->stepLastTime = delayed_by_us(now, 1); // следующий шаг сделаем микросекундой позже чем обычно
            state->errorAccumulator-=SCALE; // и вычтем из аккумулятора ошибки, что это накопление мы скомпенсировали смещением времени следующего шага
        } else {
            state->stepLastTime = now;
        }
        return true;
}

// вызывается редко и на другом ядре
void StepperMotorController::setSpeed(MotorState* state, int32_t speed){
    if (speed==0) {state->stepInterval=0; return;}
    state->direction = speed>0;
    double stepInterval = (double)1e6/abs(speed);
    uint32_t intPart = (uint32_t)stepInterval;
    double fracPart = stepInterval - (double)intPart; 
    state->stepInterval = intPart;
    state->errorIncrement = static_cast<uint32_t>(fracPart * DSCALE);
    printf("speed=%d interval=%d.%d\n", speed, state->stepInterval, state->errorIncrement);
}


void StepperMotorController::doSteps(uint8_t doStepMask){
    uint8_t lBits = (MOTOR_STEP_INVERT ? (~doStepMask) : doStepMask) & 0x0F;
    uint8_t stepsBits = STEPS_TEMPLATE | lBits;
    // pio_sm_put_blocking(pio0, 0, stepsBits);
}


void StepperMotorController::tick(absolute_time_t now){
    uint8_t needStepBits = 0;
    needStepBits |= (needStep(&stateX, now) & 1);
    needStepBits |= (needStep(&stateY, now) & 1) << 1;
    needStepBits |= (needStep(&stateA, now) & 1) << 2;
    needStepBits |= (needStep(&stateB, now) & 1) << 3;   
    if (needStepBits!=0) doSteps(needStepBits);

}
