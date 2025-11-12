#include "HX711_ADC.h"
#include "hw_config.h"

HX711_ADC LoadCell(HX711DT_PIN, HX711SCK_PIN);

static bool tareDone = false;
volatile int wire_tesn=-1;

void load_cell_setup(){
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(LOAD_CELL_FACTOR);
}


void load_cell_tare(){
  if (!tareDone) {
      LoadCell.tareNoDelay();
      tareDone = true;
    }
}

void update_load_cell_value(){
    if (LoadCell.update()) {
      int weight = (int)LoadCell.getData();
      if (wire_tesn!= weight){
        wire_tesn = weight;
      }
    }
}