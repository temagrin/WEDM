#ifndef LOAD_CELL_H
#define LOAD_CELL_H

#include <cstdint>
#include "HX711_ADC.h"

#define LOAD_CELL_FACTOR 1848.0

class LoadCell {
public:
    LoadCell(uint8_t dt_pin, uint8_t sck_pin);

    void setup();
    void tare();
    void update();

    int getWeight() const;

private:
    uint8_t dtPin;
    uint8_t sckPin;
    bool tareDone = false;
    int weight = -1;

};

#endif