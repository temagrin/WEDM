#include <pico/multicore.h>
#include "tusb.h"
#include "pico/stdlib.h"

#include "motor_controller.h"
#include "current_sensor.h"
#include "commandor.h"
#include "pulse_generator.h"
#include "ring_buffer.h"
#include "hw_config.h"


// интервал отправки статуса
static const uint64_t SEND_STATUS_INTERVAL = 1000;
static const uint64_t UPDATE_RX_INTERVAL = 10;

StepperMotorController motorController;
CurrentSensor currentSensor(CURRENT_SENCE_ADC_PIN);
PulseGenerator pulseGenerator(PULSE_PIN);
CommandManager commandManager(motorController, currentSensor, pulseGenerator);


void stepper_core() {
    absolute_time_t old = 0;
    while (true) {
        if (const absolute_time_t now = get_absolute_time(); old != now) {
            motorController.tick(now);
            old = now;
        }
    }
}



int main() {
    stdio_init_all();
    tusb_init();
    StepperMotorController::initMotors();
    currentSensor.start();
    while (!tud_cdc_connected()) { tight_loop_contents(); }
    sleep_ms(2000);
    multicore_launch_core1(&stepper_core);
    absolute_time_t lastSendStatusTime = 0;
    absolute_time_t lastRxTime = 0;

    // uint8_t breakValue;
    while (true) {
        tud_task();
        // motorController.checkBuffer();

        // breakValue = currentSensor.getCurrent()>>4;
        // if (breakValue>2) motorController.setBreak(breakValue);

        if (absolute_time_diff_us(delayed_by_us(lastRxTime, UPDATE_RX_INTERVAL), get_absolute_time())>=0){
            commandManager.updateRX();
            lastRxTime = get_absolute_time();
        }

        if (absolute_time_diff_us(delayed_by_us(lastSendStatusTime, SEND_STATUS_INTERVAL), get_absolute_time())>=0){
            commandManager.sendStatus();
            lastSendStatusTime = get_absolute_time();
        }
    }
}
