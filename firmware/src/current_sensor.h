#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "pico/stdlib.h"
#define BUFFER_SIZE 128

extern volatile uint16_t current;

void adc_dma_setup();

#endif