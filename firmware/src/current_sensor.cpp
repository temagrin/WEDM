#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hw_config.h"
#include "current_sensor.h"

uint16_t ping[BUFFER_SIZE];
uint16_t pong[BUFFER_SIZE];

volatile uint16_t current = 0;

int dma_chan;
bool write_ping = true;

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
    current = local_max;
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