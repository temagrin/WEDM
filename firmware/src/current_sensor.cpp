// current_sensor.cpp
#include <hardware/sync.h>
#include "current_sensor.h"

static CurrentSensor* irq_instance = nullptr;

CurrentSensor::CurrentSensor(uint adc_pin) : adc_pin_num(adc_pin) {
    irq_instance = this;
}

void CurrentSensor::start() {
    adc_init();
    adc_gpio_init(adc_pin_num);
    adc_select_input(0);  // Предполагается ADC0 => pin adc_pin_num
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
        ping,             // dst
        &adc_hw->fifo,    // src
        BUFFER_SIZE,      // count
        false             // do not start yet
    );

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, []() {
        if (irq_instance) irq_instance->handleIRQ();
    });
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_chan);
}

void CurrentSensor::handleIRQ() {
    dma_hw->ints0 = 1u << dma_chan;
    uint16_t *read_buffer = nullptr;

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
    current_value = local_max;
}

uint16_t CurrentSensor::getCurrent() const {
    bool irq_state = save_and_disable_interrupts();
    auto value = (uint16_t)current_value;
    restore_interrupts(irq_state);
    return value;
}