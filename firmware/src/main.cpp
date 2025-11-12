#include "pico/multicore.h"
#include <Wire.h>
#include "motor.h"
#include "load_cell.h"
#include "current_sensor.h"

void main_core0() {
    // подумать когда тарировать, с проволкой натянутой вручную, чтобы 0 было идеальное натяжение, или какое то значение.
    load_cell_tare();

    // типо крутятся движки перемотки проволки
    set_constant_speed(0, 200); 
    set_constant_speed(1, 150);
    
    static bool revers = false;
    while (true) {

        // эмитируем движение по координатам
        if (remaining_steps(2) == -1 && remaining_steps(3) == -1){
          if (revers){
            start_move_steps(3, 128*200, 800000);
            start_move_steps(2, -128*400, 2000000);
          } else {
            start_move_steps(3, -128*200, 1000000);
            start_move_steps(2, 128*400, 800000);
          }
          revers=!revers;
        }

        // выводим значения с датчиков для отладки
        Serial.printf("%d %d\n", wire_tesn, current); 

        //запрашивать часто не имеет смысла - HX711 на 80 герц режиме без разгона это 125мСек новое знаечние
        update_load_cell_value();

        // тут не страшно - это всего лишь командная часть.
        sleep_us(25000);
    }
}

void main_core1(){
  while (true) {
      handle_motors();
  }
}


void setup() {
    Serial.begin(921600, false);
    while (!Serial) {
      sleep_us(10);
    }
    sleep_us(100);
    init_motors();
    load_cell_setup();
    adc_dma_setup();
    multicore_launch_core1(&main_core1);  
}

void loop(){
  main_core0();
}