#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// wemos d1 mini (clone) - esp8266 (+ l298n)
// é uma abelha mas é um mini, tenho que ver de trocar isso

#define CONTROLE controle_preto_j_verm

#define LED LED_BUILTIN

#define motor_esq_m1 D0
#define motor_esq_m2 D5
#define motor_dir_m1 D6
#define motor_dir_m2 D7

#define ROBOT_H_HEADER
#include "robot.h"

bool ativar_turbo = false;
void _move(vel_t esq, vel_t dir) {
    // Serial.printf("andando %s\t", ativar_turbo? "turbo" : "não turbo");
    if (ativar_turbo) move(esq, dir);
    else              move(esq/6, dir/6);
}
void _hite(vel_t va, vel_t vb) {
    // Serial.printf("apertando=%d\t", vb);
    ativar_turbo = (vb > 0);
}
