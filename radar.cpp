#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "fft.h"
#include "servoControl.h"
#include "amplifierControl.h"
#include <cstdio>

// SPI Defines
#define SPI_PORT spi1
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11

#define ADC_INPUT 0
#define PIN_ADC 26

#define OSR 16
#define LOG2_OSR 4
#define SAMPLE_DEPTH_DS 1024
#define LOG2_SAMPLE_DEPTH_DS
#define SAMPLE_DEPTH SAMPLE_DEPTH_DS*OSR

#define SERVO_PIN 3

const float ADC_SAMPLING_FREQ = 5000.0*OSR; // 1024/5000 = 0.2 s, or 5 Hz resolution

uint16_t    adc_buffer[SAMPLE_DEPTH];
int16_t     adc_buffer_ds[SAMPLE_DEPTH_DS];
int16_t     fft_real[SAMPLE_DEPTH_DS];
int16_t     fft_imag[SAMPLE_DEPTH_DS];

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}

int main()
{
    stdio_init_all();
    Servo servo(SERVO_PIN);
    SPIAmplifier amplifier(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI);

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Initialize ADC
    adc_init();
    adc_select_input(ADC_INPUT);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false    // NOT Shift each sample to 8 bits when pushing to FIFO
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

    // add_alarm_in_ms(2000, alarm_callback, NULL, false);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    adc_run(true);

    printf("Starting\n");

    while (true) {
        printf(".");
        sleep_ms(10);
        if(!dma_channel_is_busy(dma_chan)){
            adc_run(false);
            adc_fifo_drain();
            // printf("\nADC capture done\n");

            for(int i = 0; i < SAMPLE_DEPTH_DS; i++){
                int16_t acc = 0;
                for(int j = 0; j < OSR; j++){
                    acc += ((int16_t)adc_buffer[i*OSR + j] - 2048);
                }
                // adc_buffer_ds[i] = acc;
                fft_real[i] = acc;
                fft_imag[i] = 0;
            }
            // printf("Starting next ADC capture\n");
            dma_channel_set_write_addr (dma_chan, adc_buffer, true);
            adc_run(true);

            fix_fft(fft_real, fft_imag, 10, 0);

            int32_t max = 0;
            int imax = -1;
            for(int i = 0; i < SAMPLE_DEPTH_DS/2; i++){
                int32_t abs2 = ((int32_t)fft_real[i]*fft_real[i]) + ((int32_t)fft_imag[i]*fft_imag[i]);
                if(abs2 > max){
                    max = abs2;
                    imax = i;
                }
            }

            if(max/(32768.0*32768.0) >= 0.001){
                printf("detection\n");
                printf("max %f\tfreq %f\n", max/(32768.0*32768.0), imax*5000.0/1024.0);
            }
        }
    }
}
