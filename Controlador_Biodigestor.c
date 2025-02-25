#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/bootrom.h"
#include "hardware/adc.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "math.h"


#define PWM_AGITADOR 20

#define BUTTON_A 5
#define BUTTON_B 6
#define BUTTON_BOOTSEL 22

#define LED_GREEN 11
#define LED_BLUE 12
#define LED_RED 13

#define VRX_PIN 26   
#define VRY_PIN 27

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

uint pwm_setup(int gpio){
    /* 60Hz pro motor e bombas*/
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 65535);
    pwm_set_clkdiv(slice, 32); 
    pwm_set_enabled(slice, true);
    return slice;
}

volatile uint32_t last_time;
volatile bool agitador_on;
volatile bool modo_manual;
void gpio_irq_handler(uint gpio, uint32_t event_mask) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > 500000){
        if (!gpio_get(BUTTON_BOOTSEL)) {
            last_time = current_time;
            rom_reset_usb_boot(0, 0);
        }
        if (!gpio_get(BUTTON_A)) {
            agitador_on=!agitador_on;
            last_time = current_time;
        }
        if (!gpio_get(BUTTON_B)) {
            modo_manual=!modo_manual;
            last_time = current_time;

        }
    }   
}

volatile float duty_bomba_entrada;
volatile float duty_bomba_saida;
void bomba_de_entrada(int* tanque, float duty_cycle){
    /* Bomba de lobulos */
    int rpm_entrada = 1300 * duty_cycle; // max
    int vazao = (rpm_entrada * 16)/1300; //max 16 m3 por minuto, nesse prototipo sera m3 por segundo
    (*tanque)+=vazao;
    printf("RPM ENTRADA: %d\n", rpm_entrada);
    printf("VAZAO ENTRADA: %d m³/min\n", vazao);

}

void bomba_de_saida(int* tanque, float duty_cycle){
    /* Bomba de lobulos */
    int rpm_saida = 1300 * duty_cycle; // max
    int vazao = (rpm_saida * 16)/1300; //max 16 m3 por minuto, nesse prototipo sera m3 por segundo
    (*tanque)-=vazao;
    printf("RPM SAÍDA: %d\n", rpm_saida);
    printf("VAZAO SAÍDA: %d m³/min\n", vazao);

}

volatile bool one_lapse;
bool repeating_timer_callback(struct repeating_timer *t){
    one_lapse = true;
    return true;
}

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_t ssd; // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_fill(&ssd, false); // Limpa display
    ssd1306_send_data(&ssd); // Envia os dados para o display

    adc_init();

    uint slice_agitador = pwm_setup(PWM_AGITADOR);
    uint slice_bomba_entrada = pwm_setup(LED_GREEN);
    uint slice_bomba_saida = pwm_setup(LED_BLUE);

    adc_gpio_init(VRX_PIN); 
    adc_gpio_init(VRY_PIN); 

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);


    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);

    gpio_init(BUTTON_BOOTSEL);
    gpio_set_dir(BUTTON_BOOTSEL, GPIO_IN);
    gpio_pull_up(BUTTON_BOOTSEL);


    gpio_set_irq_enabled_with_callback(BUTTON_BOOTSEL, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    bool going_up = true;
    float initial_dutycicle = 0.025;
    int tanque = 0;
    int limite_tanque = 6000;
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, repeating_timer_callback, false, &timer);
    
    int lapse = 0;
    int bomba_manual = 0;
    float duty;
    while (true) {
        pwm_set_gpio_level(PWM_AGITADOR, 0xffff*initial_dutycicle);
        if (agitador_on){
            gpio_put(LED_RED, 1);
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
        }
        else {
            gpio_put(LED_RED, 0);
        }
        if (one_lapse){
            if (modo_manual){
                adc_select_input(0); 
                uint16_t vry_value = adc_read();

                adc_select_input(1); 
                uint16_t vrx_value = adc_read(); 

                if (vry_value < 30)
                    bomba_manual = 1;
                if (vry_value > 4000)
                    bomba_manual = 2;
                
                
                if (bomba_manual != 0){
                    duty = (vrx_value/4090.0);
                }

                if (bomba_manual == 1)
                    duty_bomba_entrada = round(duty * 10)/10;
                if (bomba_manual == 2)
                    duty_bomba_saida = round(duty * 10)/10;

                if (duty_bomba_entrada - duty_bomba_saida <= 0.1){
                    duty_bomba_entrada = duty_bomba_saida;
                }
                
                printf("CIMA: saída, BAIXO: entrada\n");
                printf("bomba: %d\n", bomba_manual);
                printf("entrada: %f\n", duty_bomba_entrada);
                printf("saida: %f\n", duty_bomba_saida);
                one_lapse=!one_lapse;
            }
            else{
                // O dia tem 1440 minutos, 1440 segundos seria 24 minutos. 
                // Então para caber na explicação. 1440/24 seg = 60 seg = 1min
                // Um dia na vida real tem um minuto na simulacao.
                lapse++;
                bomba_manual = 0;
                bomba_de_entrada(&tanque, duty_bomba_entrada);
                bomba_de_saida(&tanque, duty_bomba_saida);
                pwm_set_gpio_level(LED_GREEN, 0xffff*duty_bomba_entrada);

                if (tanque >= limite_tanque){
                    duty_bomba_saida = 1;
                }
                pwm_set_gpio_level(LED_BLUE, 0xffff*duty_bomba_saida);
                printf("TANQUE: %dm³\n", tanque);
                one_lapse=!one_lapse;
                
            }
    
        }

        // DISPLAY
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "TOTAL  1000dias", 0, 0);
        ssd1306_draw_string(&ssd, "               ", 0, 8);

        char linha_helice[15];
        if (agitador_on){
            sprintf(linha_helice, "HELICE %2s", "ON");
        }
        else{
            sprintf(linha_helice, "HELICE %3s", "OFF");

        }
        ssd1306_draw_string(&ssd, linha_helice, 0, 16);
        
        ssd1306_draw_string(&ssd, "BOMBA1 ON  100 ", 0, 24);
        ssd1306_draw_string(&ssd, "BOMBA2 ON  100 ", 0, 32);

        char linha_tanque[15];
        sprintf(linha_tanque, "VOLUME %d M3", tanque);
        ssd1306_draw_string(&ssd, linha_tanque, 0, 40);

        ssd1306_draw_string(&ssd, "TRH       100d ", 0, 48);
        ssd1306_draw_string(&ssd, "Esvazia   2d10h", 0, 54);
         // Desenha uma string
        ssd1306_send_data(&ssd);
        // sleep_ms(1000);
    }
}
