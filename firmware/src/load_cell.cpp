#include "load_cell.h"


HX711_ADC* hx711_ptr = nullptr;

LoadCell::LoadCell(uint8_t dt_pin, uint8_t sck_pin)
    : dtPin(dt_pin), sckPin(sck_pin) {}

void LoadCell::setup() {
    if (!hx711_ptr) {
        hx711_ptr = new HX711_ADC(dtPin, sckPin);
    }
    hx711_ptr->begin();
    hx711_ptr->start(2000);
    hx711_ptr->setCalFactor(LOAD_CELL_FACTOR);
}

void LoadCell::tare() {
    if (!tareDone && hx711_ptr) {
        hx711_ptr->tareNoDelay();
        tareDone = true;
    }
}

void LoadCell::update() {
    if (hx711_ptr && hx711_ptr->update()) {
        int new_weight = static_cast<int>(hx711_ptr->getData());
        if (new_weight != weight) {
            weight = new_weight;
        }
    }
}

int LoadCell::getWeight() const {
    return weight;
}