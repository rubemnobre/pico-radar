#include "amplifierControl.h"
#include "stdio.h"

SPIAmplifier::SPIAmplifier(spi_inst_t* spi_port, uint cs_pin, uint sck_pin, uint mosi_pin, bool logMode) 
    : spi_port(spi_port), cs_pin(cs_pin), current_gain(1) { // Ganho inicial 1
    // Inicialização do SPI
    spi_init(spi_port, 1000 * 1000); // Frequência de 1 MHz
    spi_set_format(spi_port, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(cs_pin,   GPIO_FUNC_SPI);
    gpio_set_function(sck_pin,  GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);

    amplifierLog = logMode;

    if(amplifierLog) {
        printf("Amplifier SPI started\n");
    }

    bi_decl(bi_3pins_with_func(mosi_pin, sck_pin, cs_pin, GPIO_FUNC_SPI));

    set_gain(32);
}

void SPIAmplifier::set_gain(uint8_t gain) {
    if (gain < 1) gain = 1;
    if (gain > 32) gain = 32;

    // Conversão do ganho para o valor do registrador
    uint8_t spiBits[2];
    spiBits[0] = 0b01000000;       // Habilita escrita no registrador
    switch(gain){
        case 1: 
            spiBits[1] = 0;
            break;
        case 2:
            spiBits[1] = 1;
            break;
        case 4:
            spiBits[1] = 2;
            break;
        case 5: 
            spiBits[1] = 3;
            break;
        case 8: 
            spiBits[1] = 4;
            break;
        case 10:
            spiBits[1] = 5;
            break;
        case 16:
            spiBits[1] = 6;
            break;
        case 32:
            spiBits[1] = 7;
            
            break;
        default:
            spiBits[1] = 0;
            if(amplifierLog){
                printf("Gain option not available\n");
            }
    }

    uint16_t spiSendByte = (spiBits[0] << 8) | spiBits[1];

    // Envia o comando SPI
    spi_write16(spiSendByte);

    // Atualiza a memória do ganho atual
    current_gain = gain;

    if(amplifierLog) {
        printf("Gain set to %d\n", current_gain);
    }
}

uint8_t SPIAmplifier::get_gain() const {
    return current_gain;
}

void SPIAmplifier::set_next_gain() {
    switch(current_gain){
        case 1: 
            set_gain(2);
            break;
        case 2:
            set_gain(4);
            break;
        case 4:
            set_gain(5);
            break;
        case 5: 
            set_gain(8);
            break;
        case 8: 
            set_gain(10);
            break;
        case 10:
            set_gain(16);
            break;
        case 16:
            set_gain(32);
            break;
        case 32:
            set_gain(32);
            break;
        default:
            set_gain(1);
    }
}

void SPIAmplifier::set_previous_gain() {
    switch(current_gain){
        case 1: 
            set_gain(1);
            break;
        case 2:
            set_gain(1);
            break;
        case 4:
            set_gain(2);
            break;
        case 5: 
            set_gain(4);
            break;
        case 8: 
            set_gain(5);
            break;
        case 10:
            set_gain(8);
            break;
        case 16:
            set_gain(10);
            break;
        case 32:
            set_gain(16);
            break;
        default:
            set_gain(1);
    }
}

void SPIAmplifier::spi_write16(uint16_t data) {
    spi_write16_blocking(spi_port, &data, 1);
}

void SPIAmplifier::setAmplifierLog(bool newAmplifierLogMode) {
    amplifierLog = newAmplifierLogMode;
    if(amplifierLog) {
        printf("Amplifier log enabled\n");
    } else {
        printf("Amplifier log disabled\n");
    }
}