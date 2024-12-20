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

// Definições de SPI
#define SPI_PORT spi1
#define PIN_CS   13
#define PIN_SCK  10
#define PIN_MOSI 11
#define SPI_LOG 0

// Definição do servo
#define SERVO_PIN 3
#define SERVO_LOG 0

// Variáveis para execução da FFT
#define ADC_INPUT 0
#define PIN_ADC 26

#define OSR 16
#define LOG2_OSR 4
#define SAMPLE_DEPTH_DS 1024
#define LOG2_SAMPLE_DEPTH_DS 10
#define SAMPLE_DEPTH SAMPLE_DEPTH_DS*OSR
#define ADC_SAMPLING_FREQ 10000.0*OSR

#define N_PEAKS 5 

float peaks_mag2[N_PEAKS];
float peaks_freq[N_PEAKS];

SPIAmplifier amplifier(SPI_PORT, PIN_CS, PIN_SCK, PIN_MOSI, SPI_LOG);

Servo servo(SERVO_PIN, SERVO_LOG);
bool increasing = true;
const float step = 0.5;          // Incremento do ângulo
const float max_angle = 90;
float current_angle = 0;

uint16_t    adc_buffer[SAMPLE_DEPTH];
int16_t     adc_buffer_ds[SAMPLE_DEPTH_DS];
int16_t     fft_real[SAMPLE_DEPTH_DS];
int16_t     fft_imag[SAMPLE_DEPTH_DS];

float inline index_to_freq(int i){
    return i*(ADC_SAMPLING_FREQ/(1.0*OSR))/(1.0*SAMPLE_DEPTH_DS);
}

void step_servo(){
    float new_angle = increasing? (current_angle + step) : (current_angle - step);
    if(new_angle > max_angle){
        current_angle = (max_angle - (new_angle - max_angle));
        increasing = false;
    }else if(new_angle < 0){
        current_angle = (-new_angle);
        increasing = true;
    }else{
        current_angle = new_angle;
    }
    // printf("current angle = %d\n", (uint8_t)current_angle);
    servo.set_angle(current_angle);
}

int adc_dma_init(){
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
    adc_set_clkdiv(48e6/(ADC_SAMPLING_FREQ));

    // Initialize DMA
    int dma_chan = dma_claim_unused_channel(true);
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

    return dma_chan;
}

int64_t servo_timer(alarm_id_t id, void *user_data){
    step_servo();
    add_alarm_in_ms(20, servo_timer, NULL, false);
    return 0;
}

int main()
{
    stdio_init_all();
    
    
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        // printf("Wi-Fi init failed\n");
        return -1;
    }
    
    add_alarm_in_ms(1000, servo_timer, NULL, false);

    int dma_chan = adc_dma_init();

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    amplifier.set_gain(1);
    servo.set_angle(0);
    sleep_ms(1000);

    adc_run(true);

    // printf("Starting\n");
    int32_t maxmax = 0;

    while (true) {
        // printf(".");
        sleep_ms(10);
        if(!dma_channel_is_busy(dma_chan)){
            adc_run(false);
            adc_fifo_drain();
            int sat = 0;

            for(int i = 0; i < SAMPLE_DEPTH_DS; i++){
                int32_t acc = 0;
                for(int j = 0; j < OSR; j++){
                    if(((int32_t)adc_buffer[i*OSR + j] - 2048) >= 2000 || ((int32_t)adc_buffer[i*OSR + j] - 2048) < -2000){
                        sat = 1;
                    }
                    acc += ((int32_t)adc_buffer[i*OSR + j] - 2048);
                }
                
                fft_real[i] = (int16_t)(acc >> (LOG2_OSR - 4));
                fft_imag[i] = 0;
            }
            
            dma_channel_set_write_addr(dma_chan, adc_buffer, true);
            adc_run(true);

            fix_fft(fft_real, fft_imag, LOG2_SAMPLE_DEPTH_DS, 0);

            for(int j = 0; j < N_PEAKS; j++){
                peaks_mag2[j] = 0;
                peaks_freq[j] = -1;
            }

            int64_t max = 0;
            float fmax = -1;
            for(int i = 1; i < SAMPLE_DEPTH_DS/2; i++){
                int64_t abs2 = ((int32_t)fft_real[i]*(int32_t)fft_real[i]) + ((int32_t)fft_imag[i]*(int32_t)fft_imag[i]);
                if(abs2 > max){
                    max = abs2;
                    fmax = index_to_freq(i);
                    for(int j = 0; j < N_PEAKS-1; j++){
                        peaks_mag2[j] = peaks_mag2[j+1];
                        peaks_freq[j] = peaks_freq[j+1];
                    }
                    peaks_mag2[N_PEAKS-1] = (abs2)/(32768.0*32768.0);
                    peaks_freq[N_PEAKS-1] = fmax;
                }
            }

            printf("b\n");
            printf("a%.4f;\n", current_angle);
            printf("f%.4f;%.4f;%.4f;%.4f;%.4f;\n", peaks_freq[0], peaks_freq[1], peaks_freq[2], peaks_freq[3], peaks_freq[4]);
            printf("m%.4f;%.4f;%.4f;%.4f;%.4f;\n", peaks_mag2[0], peaks_mag2[1], peaks_mag2[2], peaks_mag2[3], peaks_mag2[4]);
            printf("e\n");
            if(sat == 1){
                printf("SAT\n");
            }
        }
    }
}
