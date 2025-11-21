// current_sensor.h
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

class CurrentSensor {
public:
    static constexpr int BUFFER_SIZE = 128;

    CurrentSensor(uint adc_pin);

    void start();
    void handleIRQ();  // Вызывать внутри IRQ обработчика DMA

    uint16_t getCurrent() const;

private:
    uint16_t ping[BUFFER_SIZE];
    uint16_t pong[BUFFER_SIZE];

    volatile uint16_t current_value = 0;

    int dma_chan = -1;
    bool write_ping = true;
    uint adc_pin_num;
};

#endif // CURRENT_SENSOR_H