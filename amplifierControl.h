#ifndef SPI_AMPLIFIER_H
#define SPI_AMPLIFIER_H

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

class SPIAmplifier {
public:
    // Construtor: Configura o SPI e os pinos associados
    SPIAmplifier(spi_inst_t* spi_port, uint cs_pin, uint sck_pin, uint mosi_pin);

    // Define o ganho do amplificador (1 a 32)
    void set_gain(uint8_t gain);

    // Retorna o ganho atual configurado
    uint8_t get_gain() const;

    void set_previous_gain();
    void set_next_gain();

private:
    spi_inst_t* spi_port; // Instância do SPI
    uint cs_pin;          // Pino de Chip Select (CS)
    uint current_gain;    // Memória do ganho atual configurado

    // Função para enviar comandos SPI
    void spi_write16(uint16_t data);
};

#endif
