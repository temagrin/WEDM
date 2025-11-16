#include "pico/multicore.h"
#include <Wire.h>

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
    while (true) {
        sleep_us(1);            
    }
}


void stepper_core(){
  while (true) 
  {   
    motorController.handleMotors();
  }
}





void setup() {
    Serial.begin(921600, false);
    while (!Serial) sleep_us(10);
    sleep_us(10000);
    motorController.initMotors();
    multicore_launch_core1(&stepper_core);
}

void loop(){
  main_core();
}