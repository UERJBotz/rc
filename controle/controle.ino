#include <Arduino.h>
#include "../comms.h"

#define MIXAR
#define SUAVIZAR
#define RECALIBRAR

#define DEBUG_INTERRUPTORES
#define DEBUG_JOY_CONV

#define BAUD_RATE 9600
#define ADC_MAX ((1<<12)-1)
#define ADC_MID (ADC_MAX/2)

//#define PWM_MAX ((1<<10)-1) /*1023*/
#define PWM_MAX 127 /*((1<<7)-1)*/

#define EIXO_X 2
#define EIXO_Y 1
#define BOTAO  9

#define INTERRUPTOR_INVERTER_ESQ_DIR 8
#define __INTERRUPTOR_MIXAR          7 /*! implementar */
#define INTERRUPTOR_ARMA             6
#define INTERRUPTOR_ARMA_SEC         5
// #define EIXO_ARMA     6
// #define EIXO_ARMA_SEC 5
//! trocar pra EIXO_ARMA[_SEC], INTERRUPTOR_ARMA[_SEC] por
////! EIXO_ARMA[_SEC], ifdef ARMA[_SEC]_DIGITAL

#define PEER_ADDR broadcast

struct par {
    union {
        struct { int16_t x, y; };
        struct { int16_t esq, dir; };
        struct { int16_t a, b; };
        int16_t arr[2];
    };
};

#ifndef RECALIBRAR
  const struct par PONTO_ZERO = { .x = 2815, .y = 2290 };
#else
  struct par PONTO_ZERO;
#endif

#ifdef INTERRUPTOR_INVERTER_ESQ_DIR
  bool inverter_esq_dir = false;
#else
  const bool inverter_esq_dir = false;
#endif

volatile bool inverter = false;
volatile unsigned long ultimo_clique = 0;
void IRAM_ATTR ao_apertar(void) {
    unsigned long t = millis();
    if ((t - ultimo_clique) > 1000) {
        inverter ^= 1;
        ultimo_clique = t;
    }
}

void setup() {
    pinMode(EIXO_X, INPUT);
    pinMode(EIXO_Y, INPUT);
    pinMode(BOTAO, INPUT);

  #if defined(INTERRUPTOR_INVERTER_ESQ_DIR)
    pinMode(INTERRUPTOR_INVERTER_ESQ_DIR, INPUT_PULLDOWN);
  #endif
  #if defined(INTERRUPTOR_INVERTER)
    pinMode(INTERRUPTOR_INVERTER, INPUT_PULLDOWN);
  #endif

  #if   defined(INTERRUPTOR_ARMA)
    pinMode(INTERRUPTOR_ARMA, INPUT_PULLDOWN);
  #elif defined(EIXO_ARMA)
    pinMode(EIXO_ARMA, INPUT);
  #endif
  #if   defined(INTERRUPTOR_ARMA_SEC)
    pinMode(INTERRUPTOR_ARMA_SEC, INPUT_PULLDOWN);
  #elif defined(EIXO_ARMA_SEC)
    pinMode(EIXO_ARMA_SEC, INPUT);
  #endif

  #ifdef RECALIBRAR
    PONTO_ZERO = analogico();
  #endif

    init_wifi();
    uint8_t* mac_addr = get_mac_addr();

    Serial.begin(BAUD_RATE);
    Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  mac_addr[0], mac_addr[1], mac_addr[2],
                  mac_addr[3], mac_addr[4], mac_addr[5]);

    Serial.printf("PONTO ZERO: %d, %d\n", PONTO_ZERO.x, PONTO_ZERO.y);

    static esp_now_peer_info_t peer {
        .channel = 0,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, PEER_ADDR, sizeof(PEER_ADDR));

    esp_err_t err = esp_now_add_peer(&peer);
    assert (err == ESP_OK);

    attachInterrupt(digitalPinToInterrupt(BOTAO), ao_apertar, HIGH);
}


void loop() {
    static const char sentinel = '\n';
    static char input[256] = {0};
    for (auto buf = input; Serial.available(); buf++) {
        *buf = (char) Serial.read();
        if (*buf == sentinel) {
            *buf = '\0';
            esp_err_t err = send_str(PEER_ADDR, input);

            //! print
            Serial.printf("%s ", input); 
            if (err == ESP_OK) Serial.printf("-> Serial: success\n");
            else               Serial.printf("-> Serial: error\n");
        }
    }

  #ifdef INTERRUPTOR_INVERTER_ESQ_DIR
    inverter_esq_dir = digitalRead(INTERRUPTOR_INVERTER_ESQ_DIR);
  #endif
  #ifdef INTERRUPTOR_INVERTER
    inverter = digitalRead(INTERRUPTOR_INVERTER);
  #endif

    struct par arma = vels_arma();
    struct par roda = vels_roda();

    char vels[255], *next = vels;
    if (inverter_esq_dir) next += sprintf(next,"%d %d ", roda.esq,roda.dir);
    else                  next += sprintf(next,"%d %d ", roda.dir,roda.esq);
    #if defined(INTERRUPTOR_ARMA) || defined(INTERRUPTOR_ARMA_SEC)
        next += sprintf(next, "%d %d", arma.a,arma.b);
    #endif
    next += sprintf(next, "\n");

    #ifdef DEBUG_INTERRUPTORES
        Serial.printf("ied:%1d, ift:%1d,%5d,%5d: ", inverter_esq_dir, inverter, arma.a, arma.b);
    #endif

    esp_err_t err = send_str(PEER_ADDR, vels);

    if (err == ESP_OK) Serial.printf("-> Joystick: success\n");
    else               Serial.printf("-> Joystick: error\n");
}

esp_err_t send_str(uint8_t addr[6], const char* str) {
    Packet msg {
        .id  = 0,
        .len = (uint8_t)strlen(str),
    };
    strcpy(msg.vels, str);

    uint8_t*  ptr = (uint8_t*)&msg;
    esp_err_t err = esp_now_send(addr, ptr, sizeof(msg));

    return err;
}

struct par vels_arma() {
    int16_t vel_arma = 0;
  #if   defined(INTERRUPTOR_ARMA)
    vel_arma = digital_to_pwm(digitalRead(INTERRUPTOR_ARMA));
  #elif defined(EIXO_ARMA)
    vel_arma = adc_to_pwm(analogRead(EIXO_ARMA));
  #endif

    int16_t vel_arma_sec = 0;
  #if   defined(INTERRUPTOR_ARMA_SEC)
    vel_arma_sec = digital_to_pwm(digitalRead(INTERRUPTOR_ARMA_SEC));
  #elif defined(EIXO_ARMA_SEC)
    vel_arma_sec = adc_to_pwm(analogRead(EIXO_ARMA_SEC));
  #endif

  return { .a = vel_arma, .b = vel_arma_sec };
}

struct par vels_roda() {
    struct par pos = analogico_corrigido();
    struct par pos_pwm = {
        .x = adc_to_pwm(pos.x),
        .y = adc_to_pwm(pos.y),
    };
    if (inverter) {
        pos_pwm.x = +pos_pwm.x;
        pos_pwm.y = -pos_pwm.y;
    }
    struct par vel = mixar(pos_pwm.x, pos_pwm.y);

    #ifdef DEBUG_JOY_CONV
        Serial.printf("%5d,%5d: ", pos.x, pos.y);
        Serial.printf("%5d,%5d: ", pos_pwm.x, pos_pwm.y);
        Serial.printf("%5d,%5d " , vel.esq, vel.dir);
    #endif
    return vel;
}

struct par analogico_corrigido() {
    struct par pos = analogico();
    pos = deadzone (pos.x, pos.y);
    pos = sinalizar(pos.x, pos.y);
    pos = suavizar (pos.x, pos.y);
    return pos;
}
struct par analogico() {
    return {
        .x = (int16_t)analogRead(EIXO_X),
        .y = (int16_t)analogRead(EIXO_Y),
    };
}

struct par deadzone(int16_t x, int16_t y) {
    const struct par zero = PONTO_ZERO;
    return {
        .x = (x > zero.x) ?
             map(x, zero.x,ADC_MAX, ADC_MAX/2, ADC_MAX) :
             map(x, 0,     zero.x,  0,       ADC_MAX/2),
        .y = (y > zero.y) ?
             map(y, zero.y,ADC_MAX, ADC_MAX/2, ADC_MAX) :
             map(y, 0,     zero.y,  0,       ADC_MAX/2),
    };
}
struct par sinalizar(int16_t x, int16_t y) {
    return {
        .x = map(x, 0,ADC_MAX, -ADC_MAX,ADC_MAX),
        .y = map(y, 0,ADC_MAX, -ADC_MAX,ADC_MAX),
    };
}
struct par suavizar(int32_t x, int32_t y) {
  #ifdef SUAVIZAR
    // isso segue a curva f(x) = ((x/MAX)²)*MAX = x²/MAX (mas com sinal)
    // olha no geogebra, comparando com f(x) = x
    return {
        .x = (int16_t)(x*abs(x)/ADC_MAX),
        .y = (int16_t)(y*abs(y)/ADC_MAX),
    };
  #else
    return { .x = (int16_t)x, .y = (int16_t)y };
  #endif
}

struct par mixar(int16_t x, int16_t y) {
  #ifdef MIXAR
    return {
        .esq = constrain(y + x, -PWM_MAX,PWM_MAX),
        .dir = constrain(y - x, -PWM_MAX,PWM_MAX),
    };
  #else
    return { .esq = x, .dir = y };
  #endif
}

int16_t digital_to_pwm(bool d) {
    return d ? PWM_MAX : -PWM_MAX;
}
int16_t adc_to_pwm(int16_t adc) {
    return map(adc, -ADC_MAX,ADC_MAX, -PWM_MAX,PWM_MAX);
}
