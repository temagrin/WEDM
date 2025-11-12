#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include <Wire.h>

#include <algorithm>
#include "HX711_ADC.h"
#include "GyverPlanner2.h"

#include "hw_config.h"

#define BUFFER_SIZE 128

Stepper<STEPPER2WIRE> stepperX(STEP_1_PIN, DIR_1_PIN, EN_1_PIN);
Stepper<STEPPER2WIRE> stepperY(STEP_2_PIN, DIR_2_PIN, EN_2_PIN);
Stepper<STEPPER2WIRE> stepperA(STEP_3_PIN, DIR_3_PIN, EN_3_PIN);
Stepper<STEPPER2WIRE> stepperB(STEP_4_PIN, DIR_4_PIN, EN_4_PIN);

GPlanner2<STEPPER2WIRE, 2> plannerXY;
GPlanner2<STEPPER2WIRE, 1> plannerA;
GPlanner2<STEPPER2WIRE, 1> plannerB;


HX711_ADC LoadCell(HX711DT_PIN, HX711SCK_PIN);

uint16_t ping[BUFFER_SIZE];
uint16_t pong[BUFFER_SIZE];
volatile uint16_t max_value = 0;
volatile uint16_t old_max_value = 0;
int old_weight = 1;

int dma_chan;
bool write_ping = true;

int32_t path[][2] = {
  {1000, 2500},
  {1600, 300},
  {2300, 2500},
  {600, 1000},
  {2700, 1000},
};
int count = 0;
int nodeAmount = sizeof(path) / 4;



void dma_handler() {
    dma_hw->ints0 = 1u << dma_chan;
    uint16_t *read_buffer;

    if (write_ping) {
        write_ping = false;
        read_buffer = ping;
        dma_channel_set_write_addr(dma_chan, pong, true);
    } else {
        write_ping = true;
        read_buffer = pong;
        dma_channel_set_write_addr(dma_chan, ping, true);
    }

    uint16_t local_max = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (read_buffer[i] > local_max) {
            local_max = read_buffer[i];
        }
    }
    max_value = local_max;
}



void overclock_setup() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
  Serial.printf("System clock: %i Mhz\n", int32_t(clock_get_hz(clk_sys))/1000/1000);
}


void adc_dma_setup(){
  adc_init();
  adc_gpio_init(CURRENT_SENCE_ADC_PIN);
  adc_select_input(0);
  adc_fifo_setup(
      true,    // Enable FIFO
      true,    // Enable DMA data request (DREQ)
      1,       // DREQ when at least 1 sample present
      false,   // No ERR bit
      false    // No shift
  );
  adc_set_clkdiv(0);
  adc_run(true);

  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config c = dma_channel_get_default_config(dma_chan);

  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);

  dma_channel_configure(dma_chan, &c,
      ping,            // dst
      &adc_hw->fifo,   // src
      BUFFER_SIZE,     // count
      false            // do not start yet
  );

  dma_channel_set_irq0_enabled(dma_chan, true);
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  irq_set_enabled(DMA_IRQ_0, true);
  dma_channel_start(dma_chan);
}


void stepper_setup(){
  stepperX.invertEn(0);
  stepperY.invertEn(0);
  stepperA.invertEn(0);
  stepperB.invertEn(0);
  plannerXY.addStepper(0, stepperX);
  plannerXY.addStepper(1, stepperY);
  plannerXY.setAcceleration(5000);
  plannerXY.setMaxSpeed(5000);
  plannerXY.setCurrent(path[0]);
  plannerXY.start();

  plannerA.addStepper(0, stepperA);
  plannerA.setAcceleration(5000);
  plannerA.setMaxSpeed(10000);
  plannerA.start();

  plannerA.setSpeed(0, 1000);
  
  plannerB.addStepper(0, stepperB);
  plannerB.setAcceleration(5000);
  plannerB.setMaxSpeed(10000);
  plannerB.start();
  plannerB.setSpeed(0, 1000);
  
}

void load_cell_setup(){
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(1848.0);
}


void serial_setup(){
  Serial.begin(921600, false);
  while (!Serial) {
    delay(10);
  }
  delay(100);
}


// Тут будем следить за током и натяжение проволки, а и наполняем планировщик командами (надо будет двойную буферизацию сделать обязательно)
void main_core0() {
  static bool tareDone = false;
  while(true) {
    if (old_max_value-10>max_value || old_max_value+10<max_value){
      Serial.printf("New ADC value: %u\n", max_value);
      plannerA.setSpeed(0, 1808+(max_value<<1));
      plannerB.setSpeed(0, 10000-(max_value<<1));
      old_max_value=max_value;
    }

    if (!tareDone) {
      LoadCell.tareNoDelay();
      tareDone = true;
      Serial.println("Taring...");
    }

    if (LoadCell.update()) {
      int weight = (int)LoadCell.getData();
      if (old_weight!= weight){
        Serial.print("Load: ");
        Serial.print(-1*weight);
        Serial.println(" g");
        old_weight = weight;
      }
    }

    if (plannerXY.available()) {
      plannerXY.addTarget(path[count], 0);
      if (++count >= sizeof(path) / 4) count = 0;
    }

  }
}

// Тут двигаемся
void main_core1() {
  while(true) {
    plannerA.tick();
    plannerB.tick();
    plannerXY.tick();
  }
}

void setup() {
  serial_setup();
  overclock_setup();
  adc_dma_setup();
  stepper_setup();
  load_cell_setup();
  multicore_launch_core1(&main_core1);
  return;
}

void loop() {
  main_core0();
}
