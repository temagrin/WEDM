// load_cell.cpp
#include "load_cell.h"


LoadCell::LoadCell(uint8_t dt_pin, uint8_t sck_pin)
    : dtPin(dt_pin), sckPin(sck_pin) {}

void LoadCell::setup() {
}

void LoadCell::tare() {
}

void LoadCell::update() {

}

int LoadCell::getWeight() const {
    return weight;
}