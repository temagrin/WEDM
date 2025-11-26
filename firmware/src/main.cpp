#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include "tusb.h"
#include "pico/stdlib.h"
#include "hw_config.h"
#include "motor_controller.h"
#include "current_sensor.h"
#include "commandor.h"
#include "pulse_generator.h"


StepperMotorController motorController;
CurrentSensor currentSensor(CURRENT_SENCE_ADC_PIN);
PulseGenerator pulseGenerator(PULSE_PIN);
CommandManager commandManager(&motorController, &currentSensor, &pulseGenerator);

void overclock() {
    vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
    busy_wait_us(10 * 1000);  // 10 ms delay
    set_sys_clock_khz(250000, true);             // Set to 250 MHz
}

void stepper_core() {
    absolute_time_t old = 0; // не чаще чем каждую микросекунду
    while (true) {
        if (const absolute_time_t now = get_absolute_time(); old != now) {
            motorController.tick(now);
            old = now;
        }
    }
}

int main() {
//    overclock();

    stdio_init_all();
    tusb_init();
    motorController.initMotors();
    currentSensor.start();
    while (!tud_cdc_connected()) { tight_loop_contents(); }
    // инит настройка HX711
    sleep_ms(2000);
    multicore_launch_core1(&stepper_core);

    while (true) {
        tud_task();
        commandManager.sendStatus();
        commandManager.parseCommand();
    }
}