#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// wemos d1 mini (clone) - esp8266 (+ l298n)
// é uma abelha mas é um mini, tenho que ver de trocar isso

#define CONTROLE controle_preto_j_verm

#define LED LED_BUILTIN

#define motor_esq_m1 D6
#define motor_esq_m2 D7
#define motor_dir_m1 D0
#define motor_dir_m2 D5

#define ROBOT_H_HEADER
#include "_robot.h"

bool turbo = false;
void _move(vel_t esq, vel_t dir) {
    // Serial.printf("andando %sturbo\t", turbo ? "" : "não ");
    if (turbo) move(esq, dir);
    else       move(esq/6, dir/6);
}
void _hite(vel_t va, vel_t vb) {
    // Serial.printf("apertando=%d %d\t", va, vb);
    turbo = (va > 0)^(vb > 0);
}
