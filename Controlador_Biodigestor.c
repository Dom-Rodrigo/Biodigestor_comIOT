#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções

#include "hardware/pwm.h"
#include "pico/bootrom.h"
#include "hardware/adc.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "math.h"

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#define WIFI_SSID "COMMON-C24Q1"                  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "aabb142536"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "192.168.1.4"                // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "rodrigo"     // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "rodrigo"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

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


/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */

//Leitura de temperatura do microcotrolador
static float read_onboard_temperature(const char unit);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publicar temperatura e duties
static void publish_temperature(MQTT_CLIENT_DATA_T *state);
static void publish_duty_saida(MQTT_CLIENT_DATA_T *state);
static void publish_duty_entrada(MQTT_CLIENT_DATA_T *state);
static void publish_tanque(MQTT_CLIENT_DATA_T *state);



// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

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
int tanque = 0;

float vazao_entrada_diaria;
void bomba_de_entrada(int* tanque, float duty_cycle){
    /* Bomba de lobulos */
    int rpm_entrada = 1300 * duty_cycle; // max
    int vazao = (rpm_entrada * 16)/1300.0; //max 16 m3 por minuto, nesse prototipo sera m3 por segundo
    vazao_entrada_diaria = vazao * 1440; // 1440min = um dia;
    (*tanque)+=vazao;
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
    INFO_printf("mqtt client starting\n");


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
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

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
    int limite_tanque = 6000 - 16;
    struct repeating_timer timer;
    add_repeating_timer_ms(1000/24, repeating_timer_callback, false, &timer);
    
    int lapse = 0;
    int dias = 0;
    int bomba_manual = 0;
    int cor;
    int pisca_volume;
    float duty;

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        panic("Failed to inizialize CYW43");
    }

 // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        panic("Failed to connect");
    }

     INFO_printf("\nConnected to Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("dns request failed");
    }

    while (true) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
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
                if ((duty_bomba_entrada < duty_bomba_saida) && tanque > 0){
                    pwm_set_gpio_level(BUZZER_A, 0xffff*1);
                    pisca_volume = true;

                }
                else{
                    pwm_set_gpio_level(BUZZER_A, 0);
                    pisca_volume = false;

                }

                if (tanque > limite_tanque){
                    if (tanque > limite_tanque+16){
                        duty_bomba_entrada = 0;
                        duty_bomba_saida = 1; // Sai até remover o excesso BUZZER vai aqui
                        pwm_set_gpio_level(BUZZER_A, 0xffff*1);
                        sleep_us(100);
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
            sprintf(linha_helice, "HELICE  ON");
        }
        else{
            sprintf(linha_helice, "HELICE  OFF");

        }
        ssd1306_draw_string(&ssd, linha_helice, 0, 16);


        // LINHA QUE MOSTRA ESTADO DA BOMBA COM DUTYCYCLE
        char linha_entrada[15];
        if (duty_bomba_entrada > 0.001){
            sprintf(linha_entrada, "ENTRADA ON %.0f", duty_bomba_entrada*100);
        }
        else{
            sprintf(linha_entrada, "ENTRADA OFF 0");

        }
        ssd1306_draw_string(&ssd, linha_entrada, 0, 24);


        // LINHA QUE MOSTRA ESTADO DA BOMBA COM DUTYCYCLE
        char linha_saida[15];
        if (duty_bomba_saida > 0.001){
            sprintf(linha_saida, "SAIDA   ON %.0f", duty_bomba_saida*100);
        }
        else{
            sprintf(linha_saida, "SAIDA   OFF 0");

        }
        ssd1306_draw_string(&ssd, linha_saida, 0, 32);

        // LINHA QUE MOSTRA O VOLUME DE SUBSTRATO NO TANQUE BIODIGESTOR
        char linha_tanque[15];
        sprintf(linha_tanque, "VOLUME  %d M3", tanque);
        if (pisca_volume){
            cor++;
            if (cor % 2 == 0)
                ssd1306_draw_string(&ssd, linha_tanque, 0, 40);
            else
                ssd1306_draw_string(&ssd, "              ", 0, 40);

        }
        else {
            ssd1306_draw_string(&ssd, linha_tanque, 0, 40);
        }



        // VOLUME TOTAL/Tempo DE RETENÇÃO hidráulica = VOLUME DIARIO RECOMENDADO
        char linha_trh[15];
        sprintf(linha_trh, "TRH %.0f d", vazao_entrada_diaria/tanque);
        ssd1306_draw_string(&ssd, linha_trh, 0, 48);
        // ssd1306_draw_string(&ssd, "Esvazia   2d10h", 0, 54);
         // Desenha uma string
        ssd1306_send_data(&ssd);
        // sleep_ms(1000);
    }
    INFO_printf("mqtt client exiting\n");

}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

static void publish_duty_entrada(MQTT_CLIENT_DATA_T *state){
    static float duty_entrada_antigo;
    const char *duty_entrada_key = full_topic(state, "/duty_entrada");
    if (duty_bomba_entrada != duty_entrada_antigo){
        duty_entrada_antigo = duty_bomba_entrada;
        char duty_str[16];
        snprintf(duty_str, sizeof(duty_str), "%.2f", duty_bomba_entrada);
        INFO_printf("Publishing %s to %s\n", duty_str, duty_entrada_key);
        mqtt_publish(state->mqtt_client_inst, duty_entrada_key, duty_str, strlen(duty_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}
static void publish_duty_saida(MQTT_CLIENT_DATA_T *state){
    static float duty_saida_antigo;
    const char *duty_saida_key = full_topic(state, "/duty_saida");
    if (duty_bomba_saida != duty_saida_antigo){
        duty_saida_antigo = duty_bomba_saida;
        char duty_str[16];
        snprintf(duty_str, sizeof(duty_str), "%.2f", duty_bomba_saida);
        INFO_printf("Publishing %s to %s\n", duty_str, duty_saida_key);
        mqtt_publish(state->mqtt_client_inst, duty_saida_key, duty_str, strlen(duty_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}
static void publish_tanque(MQTT_CLIENT_DATA_T *state){
    static int tanque_antigo;
    const char *tanque_key = full_topic(state, "/tanque");
    if (tanque != tanque_antigo){
        tanque_antigo = tanque;
        char tanque_str[16];
        snprintf(tanque_str, sizeof(tanque_str), "%d", tanque);
        INFO_printf("Publishing %s to %s\n", tanque_str, tanque_key);
        mqtt_publish(state->mqtt_client_inst, tanque_key, tanque_str, strlen(tanque_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}


// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    publish_temperature(state);
    publish_duty_entrada(state);
    publish_duty_saida(state);
    publish_tanque(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}
