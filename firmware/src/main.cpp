#include <SerialUSB.h>
#include "pico/multicore.h"
#include "hw_config.h"
#include "motor.h"
#include "motor_controller.h"
#include "motor_planner.h"
#include "motor_drivers.h"

StepperMotor motorA(STEP_1_PIN, DIR_1_PIN, EN_1_PIN, LV8729);
StepperMotor motorB(STEP_2_PIN, DIR_2_PIN, EN_2_PIN, LV8729);
StepperMotor motorX(STEP_3_PIN, DIR_3_PIN, EN_3_PIN, LV8729);
StepperMotor motorY(STEP_4_PIN, DIR_4_PIN, EN_4_PIN, LV8729);

StepperMotorController motorController({&motorX, &motorY, &motorA, &motorB});
StepperMotorPlanner motorPlanner(&motorX, &motorY);

uint8_t task_id = 0;

int32_t tasks[6][3] = {
  {1000,2000,1000},
  {500,4000,1000},
  {1500,1500,10000},
  {100000,200000,100000},
  {500000,400000,100000},
  {150000,150000,100000},
};

void new_task(){
  if (task_id<6){
  sleep_us(1000000);
  Serial.println();
  Serial.println();
  Serial.println("----------");
  Serial.printf("Start new test case %d\n", task_id);
  Serial.println("----------");
  motorPlanner.reset();
  motorPlanner.moveTo(tasks[task_id][0],tasks[task_id][1],tasks[task_id][2]);
  }
  if (task_id<7){
    task_id++;
  }
  if (task_id>6){
    motorX.setPower(false);
    motorY.setPower(false);
  }
}

void main_core() {
    sleep_us(100000);

    // motorA.setPower(true);
    // motorB.setPower(true);
    // motorA.setInfininityRorationSpeed((int32_t)128*200*2);
    // motorB.setInfininityRorationSpeed((int32_t)-128*200*4);
    
    // тестовое задание
    motorX.setCalcSteps(true);
    motorY.setCalcSteps(true);
    motorX.setPower(true);
    motorY.setPower(true);
    
    while (true) {
        if (motorPlanner.ready()) new_task();
        motorPlanner.handlePlaner();
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