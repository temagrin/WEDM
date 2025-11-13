#include "pico/multicore.h"
#include <Wire.h>
#include "motor.h"
#include "load_cell.h"
#include "current_sensor.h"
#include "pulse_generator.h"
#include "xy_stepper_planner.h"
#include "hw_config.h"

LoadCell loadCell(HX711DT_PIN, HX711SCK_PIN);

CurrentSensor currentSensor(CURRENT_SENCE_ADC_PIN);

PulseGenerator pulse(PULSE_PIN);

StepperMotor motorA(STEP_1_PIN, DIR_1_PIN, EN_1_PIN);
StepperMotor motorB(STEP_2_PIN, DIR_2_PIN, EN_2_PIN);
StepperMotor motorX(STEP_3_PIN, DIR_3_PIN, EN_3_PIN);
StepperMotor motorY(STEP_4_PIN, DIR_4_PIN, EN_4_PIN);

StepperMotorController motorController;
XYStepperPlanner planner(motorX, motorY);


void main_core0() {
    static bool revers = false;
    while (true) {
        loadCell.update();
        int wire_tesn = loadCell.getWeight();
        uint16_t current = currentSensor.getCurrent();

        // выводим значения с датчиков для отладки
        Serial.printf("%d %d\n", wire_tesn, current); 
        // тригаем планировщик
        if (current > 2000 ){
            planner.moveTo(50000, 30000, 1000000);
        }
        if (current < 1000 ){
            planner.moveTo(0, 0, 1000000);
        }
        // тут не страшно - это всего лишь командная часть.
        sleep_us(25000);
    }
}

void main_core1(){
  while (true) {
      motorController.handleMotors();
      planner.update();
  }
}


void setup() {
    Serial.begin(921600, false);
    while (!Serial) {
      sleep_us(10);
    }
    sleep_us(100);

    loadCell.setup();
    loadCell.tare();

    currentSensor.setup();

    pulse.setPulseWidth(5);    // 5 мкс
    pulse.setPausePeriod(100); // 100 мкс   можно вместо периода паузы указать сразу частоту pulse.setFrequency(10000) - 10кГц)
    pulse.enable(true);

    motorController.addMotor(motorA);
    motorController.addMotor(motorB);
    motorController.addMotor(motorX);
    motorController.addMotor(motorY);
    motorController.init();
    
    motorA.setConstantSpeed(200);
    motorB.setConstantSpeed(-250);
    
    multicore_launch_core1(&main_core1);  
}

void loop(){
  main_core0();
}