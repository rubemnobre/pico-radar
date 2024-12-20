#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "hardware/pwm.h"

class Servo {
public:
    Servo(uint gpio_pin, bool logMode);
    
    // Define o ângulo do servo (0 a 180 graus)
    void set_angle(float angle);

    // Retorna o último ângulo configurado
    float get_angle() const;

    void setServoLog(bool newServoLogMode);

private:
    uint gpio_pin;
    uint slice_num;
    float current_angle; // Memória para o último ângulo definido

    bool servoLog = 0;

    static const uint16_t SERVO_MIN_US = 666;
    static const uint16_t SERVO_MAX_US = 2666;
    static const uint PWM_FREQ_HZ = 50;
};

#endif // SERVO_CONTROL_H
