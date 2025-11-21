#include <cstdio>
#include <pico/stdlib.h>
#include <pico/stdio_usb.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/pll.h>
#include <hardware/vreg.h>

#include "hw_config.h"
#include "motor_controller.h"
#include "current_sensor.h"


void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
}

StepperMotorController motorController;
CurrentSensor currentSensor(CURRENT_SENCE_ADC_PIN);

void stepper_core(){
    
    absolute_time_t old = 0;
    while(true) {
        if (const absolute_time_t now = get_absolute_time(); old!=now){
            motorController.tick(now);
            old=now;
        }
    } 
}

int main() {
    overclock();
    stdio_init_all();
    
    while (!stdio_usb_connected()) {
        tight_loop_contents(); 
    }
    
    sleep_ms(2000);
    printf("Hello world\n\n");
    
    currentSensor.start();
    motorController.initMotors();
    motorController.setPowerXY(true);
    motorController.setPowerA(true);
    motorController.setPowerB(true);
    
    multicore_launch_core1(&stepper_core);


    // uint16_t currentOld = 0;
    // int16_t current=0;
    //
    while(true) {
        motorController.setDirA(true);
        sleep_ms(10);
        motorController.setDirA(false);
        sleep_ms(10);
        // current = currentSensor.getCurrent()-25;
        // if (currentOld-10>current || currentOld+10<current){
        //     currentOld = current;
        //     motorController.setSpeedA(current*24+80000);
        //     motorController.setSpeedB(current*24+80000);
        //     motorController.setSpeedX(current*24+80000);
        //     motorController.setSpeedY(current*24+80000);
        // }
        // sleep_ms(25);

        tight_loop_contents();
    } 
}