#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include <Wire.h>
#include <algorithm>

#include "hw_config.h"

void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
}


void main_core0() {
  Serial.printf("starting servo controll loops on core 0...\n");
  while(true) {
  delay(1);
  }
}

void main_core1() {
  Serial.printf("starting servo controll loops on core 1...\n");
  while(true) {
  delay(1);
  }
}

void setup() {
  // stdio_init_all();  // Initializes USB or UART stdio
  overclock();
  Serial.begin(921600, false);
  while(!Serial);
  delay(100);
  Serial.printf("System clock: %i Mhz\n", int32_t(clock_get_hz(clk_sys))/1000/1000);
  multicore_launch_core1(&main_core1);
  return;
}

void loop() {
  main_core0();
}
