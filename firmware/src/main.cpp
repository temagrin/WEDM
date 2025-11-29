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
#include "ring_buffer.h"



// интервал отправки статуса
static const uint64_t SEND_STATUS_INTERVAL = 1000;

CommandRingBuffer queue;
const absolute_time_t now = get_absolute_time();
MotorState stateA = {now,0, 0, 0, false, false, 0, true};
MotorState stateB = {now,0, 0, 0, false, false, 0, true};
MotorState stateX = {now,0, 0, 0, false, true, 0, true};
MotorState stateY = {now,0, 0, 0, false, true, 0, true};

StepperMotorController motorController(queue, stateA, stateB, stateX, stateY);
CurrentSensor currentSensor(CURRENT_SENCE_ADC_PIN);
PulseGenerator pulseGenerator(PULSE_PIN);
CommandManager commandManager(motorController, currentSensor, pulseGenerator, queue);

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
    // TODO инит настройка HX711

    while (!tud_cdc_connected()) { tight_loop_contents(); }

    sleep_ms(2000);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, true);

    gpio_init(20);
    gpio_set_dir(20, GPIO_OUT);
    gpio_put(20, true);

    sleep_ms(1000);
    multicore_launch_core1(&stepper_core);
    absolute_time_t lastSendStatusTime = 0;
    gpio_put(22, false);
    gpio_put(20, false);
    uint8_t breakValue;
    while (true) {
        tud_task();
        commandManager.updateRX();
        motorController.checkBuffer();
        breakValue = currentSensor.getCurrent()>>4;
        if (breakValue>2) motorController.setBreak(breakValue);
        if (absolute_time_diff_us(delayed_by_us(lastSendStatusTime, SEND_STATUS_INTERVAL), get_absolute_time())>=0){
            commandManager.sendStatus();
            lastSendStatusTime = get_absolute_time();
        }
    }
}
