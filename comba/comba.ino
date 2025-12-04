#include <Arduino.h>

//#define CONTROLE controle_branco_1_joystick
#define VEL_MAX 127
#include "gesonel.h" // esse include muda os pinos do robô
#include "_robot.h"  // esse tem uma implementação genérica dos robôs
#include "_comms.h"

#define BAUD_RATE 115200
#define IDX_VEL 0
#define IDX_ESC 1

#define memeql(a,b,sz) (memcmp(a,b,sz) == 0)
#define LEN(arr) (sizeof(arr)/sizeof(*arr))

struct vel { int16_t esq = 0, dir = 0; }; //! nomes
union vels {
    int16_t   raw[6];
    struct vel of[3];
};

union vels str_to_vels(char *const text, uint8_t len) {
    const char sep = ' ';

    uint8_t v = 1, seps[6] = {0};
    for (size_t i = 0; i < len; i++) {
        if (text[i] == sep) seps[v++] = i;

        if (text[i] == '\0') break;
        if (v  >= LEN(seps)) break;
    }

    union vels vels{0};
    for (size_t i = 0; i < LEN(seps); i++) {
        vels.raw[i] = atoi(&text[seps[i]]);
    }
    return vels;
}

union vels vels{0};
unsigned long t_recv = 0;
void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, esp_now_len_t len) {
    Packet* msg = (Packet*) (void*)data;
    if (msg->id != 0) return; //!

  #ifdef CONTROLE
    if (!memeql(info->src_addr, CONTROLE, sizeof(CONTROLE))) return;
  #else
    #warning "aceitando conexão de qualquer controle"
  #endif

    t_recv = millis();
    vels = str_to_vels(msg->vels, msg->len);
}

void setup() {
    Serial.begin(BAUD_RATE);

    robot_setup();
    espnow_setup(on_recv);
}

void loop() {
    struct vel vel_rodas{0}, vel_esc{0}, extra{0};
    if ((millis() - t_recv) < 1000) {
        vel_rodas = vels.of[IDX_VEL];
        vel_esc   = vels.of[IDX_ESC];
        extra     = vels.of[2]; //! mágico
    }

    move(vel_rodas.esq, vel_rodas.dir);
    hite(vel_esc.esq);

    //! print
    Serial.printf("vels %d %d, esc %d %d, n/a %d %d\n",
                  vel_rodas.esq, vel_rodas.dir,
                  vel_esc.esq,   vel_esc.dir,
                  extra.esq,     extra.dir);

    yield();
}
