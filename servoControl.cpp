#include "servoControl.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "stdio.h"

// Construtor: Inicializa o servo na GPIO especificada
Servo::Servo(uint gpio_pin, bool logMode) : gpio_pin(gpio_pin), current_angle(0) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f); // Ajuste do divisor
    pwm_init(slice_num, &config, true);
    servoLog = logMode;
    if(servoLog) {
        printf("Servo started\n");
    }
    set_angle(0);
}

// Define o ângulo do servo (0 a 180 graus)
void Servo::set_angle(float angle) {
    if (angle > 180) angle = 180;

    uint16_t pulse_width = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
    uint32_t clock_rate = clock_get_hz(clk_sys) / 64;
    uint16_t top_value = clock_rate / PWM_FREQ_HZ;

    pwm_set_wrap(slice_num, top_value);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), (pulse_width * top_value) / 20000);

    current_angle = angle; // Atualiza a memória com o ângulo atual

    if(servoLog) {
        printf("Servo angle set to %d degrees.\n", current_angle);
    }
    
}

// Retorna o último ângulo configurado
float Servo::get_angle() const {
    if(servoLog){
        printf("Servo angle is %f degrees.\n", current_angle);
    }
    return current_angle;
}

void Servo::setServoLog(bool newServoLogMode) {
    servoLog = newServoLogMode;
    if(servoLog) {
        printf("Servo log enabled\n");
    } else {
        printf("Servo log disabled\n");
    }
}