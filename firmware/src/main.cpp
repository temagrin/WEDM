#include "pico/multicore.h"
#include <Wire.h>

#include "hw_config.h"
#include "motor.h"
#include "motor_controller.h"
#include "motor_drivers.h"

StepperMotor motorX(STEP_1_PIN, DIR_1_PIN, EN_1_PIN, LV8729);
StepperMotor motorY(STEP_2_PIN, DIR_2_PIN, EN_2_PIN, LV8729);
StepperMotor motorA(STEP_3_PIN, DIR_3_PIN, EN_3_PIN, LV8729);
StepperMotor motorB(STEP_4_PIN, DIR_4_PIN, EN_4_PIN, LV8729);

StepperMotorController motorController(&motorX, &motorY, {&motorA, &motorB});


void main_core() {
    sleep_us(1000000);
    motorA.setPower(true);
    motorB.setPower(true);
    while (true) {
        Serial.println("Tuda");
        motorController.setMotorSpeed(0, 128*200*1);
        motorController.setMotorSpeed(1, 128*200*2);
        sleep_us(2000000);
        Serial.println("Suda");
        motorController.setMotorSpeed(0, -128*100*1);
        motorController.setMotorSpeed(1, -128*100*1);
        sleep_us(2000000);
        Serial.println("Stope");    
        motorController.setMotorSpeed(0, 0);
        motorController.setMotorSpeed(1, 0);
        sleep_us(2000000);
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