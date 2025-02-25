#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define PWM_AGITADOR 22

uint pwm_setup(){
    gpio_set_function(PWM_AGITADOR, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_AGITADOR);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 38.4);
    pwm_init(slice, &config, true);

    return slice;
}

int main()
{
    stdio_init_all();
    uint slice = pwm_setup();


    bool going_up = true;
    float initial_dutycicle = 0.025;
    while (true) {
        pwm_set_gpio_level(PWM_AGITADOR, 0xffff*initial_dutycicle);
        sleep_ms(10);
        if (going_up){
            initial_dutycicle= initial_dutycicle - 0.00025;
            if (initial_dutycicle <= 0.025)
                going_up=false;
        }
        else
        {
            initial_dutycicle= initial_dutycicle + 0.00025;
            if (initial_dutycicle >= 0.12)
                going_up=true;
        } 
        // Regra de três para encontrar a porcentagem do dutycicle
        // 2400ms -> 0.12 ou 12%
        // 5ms -> 1/4000 = 0.00025 ou 0.025%
    }
}
