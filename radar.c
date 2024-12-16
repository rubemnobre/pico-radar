#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/cyw43_arch.h"
#include "fft.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

#define ADC_INPUT 0
#define PIN_ADC 26

#define OSR 16
#define LOG2_OSR 4
#define SAMPLE_DEPTH_DS 1024
#define LOG2_SAMPLE_DEPTH_DS
#define SAMPLE_DEPTH SAMPLE_DEPTH_DS*OSR

const float ADC_SAMPLING_FREQ = 10000.0*OSR;

// uint8_t out_buf[BUF_LEN], in_buf[BUF_LEN];

uint16_t    adc_buffer[SAMPLE_DEPTH];
int16_t     adc_buffer_ds[SAMPLE_DEPTH_DS];
int16_t     fft_buffer[SAMPLE_DEPTH_DS];

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

int main()
{
    stdio_init_all();
    printf("Boot\n");

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // Initialize ADC
    adc_init();
    adc_select_input(ADC_INPUT);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false     // NOT Shift each sample to 8 bits when pushing to FIFO
    );

    adc_gpio_init(PIN_ADC);
    adc_set_clkdiv(0);

    // Initialize DMA
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);
    
    dma_channel_configure(dma_chan, &cfg,
        adc_buffer,    // dst
        &adc_hw->fifo,  // src
        SAMPLE_DEPTH,  // transfer count
        true            // start immediately
    );

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_ms(2000, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    adc_run(true);

    printf("Starting\n");

    while (true) {
        printf(".");
        sleep_ms(10);
        if(!dma_channel_is_busy(dma_chan)){
            adc_run(false);
            adc_fifo_drain();
            printf("\nADC capture done\n");

            for(int i = 0; i < SAMPLE_DEPTH_DS; i++){
                int16_t acc = 0;
                for(int j = 0; j < OSR; j++){
                    acc += ((int16_t)adc_buffer[i*OSR + j] - 2048);
                }
                adc_buffer_ds[i] = acc;
                fft_buffer[i] = acc;
            }

            // adc12_to_int16(adc_buffer_ds, fft_buffer, SAMPLE_DEPTH_DS);
            fix_fftr(fft_buffer, 10, 0);

            for(int i = 0; i < SAMPLE_DEPTH_DS; i++){
                printf("%d;%d\n", fft_buffer[i], adc_buffer_ds[i]);
            }
            sleep_ms(5000);
            printf("Starting next ADC capture\n");
            dma_channel_set_write_addr (dma_chan, adc_buffer, true);
            adc_run(true);
        }
    }
}
