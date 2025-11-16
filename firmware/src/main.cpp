#include <SerialUSB.h>
#include "pico/multicore.h"
#include "hw_config.h"
#include "motor.h"
#include "motor_controller.h"
#include "motor_drivers.h"

StepperMotor motorA(STEP_1_PIN, DIR_1_PIN, EN_1_PIN, LV8729);
StepperMotor motorB(STEP_2_PIN, DIR_2_PIN, EN_2_PIN, LV8729);
StepperMotor motorX(STEP_3_PIN, DIR_3_PIN, EN_3_PIN, LV8729);
StepperMotor motorY(STEP_4_PIN, DIR_4_PIN, EN_4_PIN, LV8729);

StepperMotorController motorController(&motorX, &motorY, {&motorA, &motorB});


void main_core() {
    sleep_us(100000);
    motorX.setPower(true);
    motorY.setPower(true);
    
    // тестовое задание
    motorController.moveTo(128*200*12, 128*200*10, 0, 128*200*2, 0);

    while (true) {
        // также тут вызываем паузу по замыканию, резюме, по размыканию, корректируем натяжение, парсим Gcode, собираем буфер команд для motorController.moveT и прочии штуки дрюки.
        motorController.handlePlaner();
        sleep_us(200000);
    }
}


void stepper_core(){
  while (true) 
  {   
    // это ядро ничем дополнительным не нагружаем - тут двигатели шагают - важна каждая микросекунда
    motorController.handleMotors();
  }
}


void setup() {
    Serial.begin(921600, false);
    while (!Serial) sleep_us(10);
    sleep_us(10000);
    Serial.println("Start");
    motorController.initMotors();
    multicore_launch_core1(&stepper_core);
}

void loop(){
  main_core();
}