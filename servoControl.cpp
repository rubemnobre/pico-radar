#include "servoControl.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Construtor: Inicializa o servo na GPIO especificada
Servo::Servo(uint gpio_pin) : gpio_pin(gpio_pin), current_angle(0) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f); // Ajuste do divisor
    pwm_init(slice_num, &config, true);
    set_angle(0); // Inicializa o servo em 0 graus
}

// Define o ângulo do servo (0 a 180 graus)
void Servo::set_angle(uint8_t angle) {
    if (angle > 180) angle = 180;

    uint16_t pulse_width = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
    uint32_t clock_rate = clock_get_hz(clk_sys) / 64;
    uint16_t top_value = clock_rate / PWM_FREQ_HZ;

    pwm_set_wrap(slice_num, top_value);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), (pulse_width * top_value) / 20000);

    current_angle = angle; // Atualiza a memória com o ângulo atual
}

// Retorna o último ângulo configurado
uint8_t Servo::get_angle() const {
    return current_angle;
}