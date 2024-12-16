#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "hardware/pwm.h"

class Servo {
public:
    Servo(uint gpio_pin);
    
    // Define o ângulo do servo (0 a 180 graus)
    void set_angle(uint8_t angle);

    // Retorna o último ângulo configurado
    uint8_t get_angle() const;

private:
    uint gpio_pin;
    uint slice_num;
    uint8_t current_angle; // Memória para o último ângulo definido

    static const uint16_t SERVO_MIN_US = 500;
    static const uint16_t SERVO_MAX_US = 2500;
    static const uint PWM_FREQ_HZ = 50;
};

#endif // SERVO_CONTROL_H
