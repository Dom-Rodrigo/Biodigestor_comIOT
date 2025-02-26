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

#define BUZZER_A 10

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
    pwm_set_wrap(slice, 0xffff);
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

volatile float duty_bomba_entrada = 0.0;
volatile float duty_bomba_saida = 0.0;
float vazao_entrada_diaria;
void bomba_de_entrada(int* tanque, float duty_cycle){
    /* Bomba de lobulos */
    int rpm_entrada = 1300 * duty_cycle; // max
    int vazao = (rpm_entrada * 16)/1300.0; //max 16 m3 por minuto, nesse prototipo sera m3 por segundo
    vazao_entrada_diaria = vazao * 1440; // 1440min = um dia;
    (*tanque)+=vazao;
    if (*tanque <= 0)
        *tanque=0;
}

void bomba_de_saida(int* tanque, float duty_cycle){
    /* Bomba de lobulos */
    int rpm_saida = 1300 * duty_cycle; // max
    int vazao = (rpm_saida * 16)/1300.0; //max 16 m3 por minuto, nesse prototipo sera m3 por segundo
    (*tanque)-=vazao;
    if (*tanque <= 0)
        *tanque=0;
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
    uint slice_bomba_saida = pwm_setup(LED_BLUE);  // 60Hz
    uint slice_buzzer = pwm_setup(BUZZER_A); // 60hz

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
    int limite_tanque = 6000 - 16;
    struct repeating_timer timer;
    add_repeating_timer_ms(1000/24, repeating_timer_callback, false, &timer);
    
    int lapse = 0;
    int dias = 0;
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
                    bomba_manual = 2;
                if (vry_value > 4000)
                    bomba_manual = 1;
                
                
                if (bomba_manual != 0){
                    duty = (vrx_value/4090.0);
                }

                if (bomba_manual == 1)
                    duty_bomba_entrada = round(duty * 10)/10;
                if (bomba_manual == 2)
                    duty_bomba_saida = round(duty * 10)/10;

                one_lapse=!one_lapse;
            }
            else{
                // O dia tem 1440 minutos, 1440 segundos seria 24 minutos. 
                // Então para caber na explicação. 1440/24 seg = 60 seg = 1min
                // Um dia na vida real tem um minuto na simulacao.
                lapse++;
                if (lapse % 1440 == 0) {
                    dias++;
                    if (tanque == 0){
                        dias = 0; // Reseta o numero de dias
                    }
                }

                bomba_manual = 0;
                pwm_set_gpio_level(LED_GREEN, 0xffff*duty_bomba_entrada);
                pwm_set_gpio_level(LED_BLUE, 0xffff*duty_bomba_saida);
                bomba_de_entrada(&tanque, duty_bomba_entrada);
                bomba_de_saida(&tanque, duty_bomba_saida);
                if (duty_bomba_entrada < duty_bomba_saida && tanque > 0){
                    pwm_set_gpio_level(BUZZER_A, 0xffff*1);
                    sleep_ms(500); // Mantém por 500ms antes de desligar

                }
                else{
                    pwm_set_gpio_level(BUZZER_A, 0);

                }

                if (tanque > limite_tanque){
                    if (tanque > limite_tanque+16){
                        duty_bomba_entrada = 0;
                        duty_bomba_saida = 1; // Sai até remover o excesso BUZZER vai aqui
                        pwm_set_gpio_level(BUZZER_A, 0xffff*1);
                    }
                    else {
                        duty_bomba_saida = 1; // Estabiliza
                        duty_bomba_entrada = 1;
                    }
                }

                one_lapse=!one_lapse;
                pwm_set_gpio_level(BUZZER_A, 0xffff*0);

                
            }
    
        }

        // DISPLAY
        ssd1306_fill(&ssd, false);

        // LINHA QUE MOSTRA O TOTAL DE DIAS COM QUE O TANQUE RECEBEU O SUBSTRATO
        char linha_dias[15];
        sprintf(linha_dias, "TOTAL %4dd", dias);
        ssd1306_draw_string(&ssd, linha_dias, 0, 0);

        ssd1306_draw_string(&ssd, "               ", 0, 8);

        // LINHA QUE DIZ SE A HÉLICE DO AGITADOR ESTÁ FUNCIONANDO
        char linha_helice[15];
        if (agitador_on){
            sprintf(linha_helice, "HELICE ON");
        }
        else{
            sprintf(linha_helice, "HELICE OFF");

        }
        ssd1306_draw_string(&ssd, linha_helice, 0, 16);


        // LINHA QUE MOSTRA ESTADO DA BOMBA COM DUTYCYCLE
        char linha_entrada[15];
        if (duty_bomba_entrada > 0.001){
            sprintf(linha_entrada, "ENTRAD ON %.0f", duty_bomba_entrada*100);
        }
        else{
            sprintf(linha_entrada, "ENTRAD OFF 0");

        }
        ssd1306_draw_string(&ssd, linha_entrada, 0, 24);


        // LINHA QUE MOSTRA ESTADO DA BOMBA COM DUTYCYCLE
        char linha_saida[15];
        if (duty_bomba_saida > 0.001){
            sprintf(linha_saida, "SAIDA  ON %.0f", duty_bomba_saida*100);
        }
        else{
            sprintf(linha_saida, "SAIDA  OFF 0");

        }
        ssd1306_draw_string(&ssd, linha_saida, 0, 32);

        // LINHA QUE MOSTRA O VOLUME DE SUBSTRATO NO TANQUE BIODIGESTOR
        char linha_tanque[15];
        sprintf(linha_tanque, "VOLUME %d M3", tanque);
        ssd1306_draw_string(&ssd, linha_tanque, 0, 40);


        // VOLUME TOTAL/Tempo DE RETENÇÃO hidráulica = VOLUME DIARIO RECOMENDADO
        char linha_trh[15];
        sprintf(linha_trh, "TRH %.0f", vazao_entrada_diaria/tanque);
        ssd1306_draw_string(&ssd, linha_trh, 0, 48);
        // ssd1306_draw_string(&ssd, "Esvazia   2d10h", 0, 54);
         // Desenha uma string
        ssd1306_send_data(&ssd);
        // sleep_ms(1000);
    }
}
