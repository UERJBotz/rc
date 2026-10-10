#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Wemos D1 mini (clone) - esp8266 (+ l298n)
//! chamar de abelhas em vez de soldados (brincadeira com a vespa da robocore)

#define CONTROLE controle_roxo_j_verm

#define LED LED_BUILTIN

#define motor_esq_m1 D1
#define motor_esq_m2 D2
#define motor_dir_m1 D3
#define motor_dir_m2 D4

bool turbo = false;
// void _move(vel_t esq, vel_t dir) {
//     // Serial.printf("andando %sturbo\t", turbo ? "" : "não ");
//     if (turbo) move(esq, dir);
//     else       move(esq/6, dir/6);
// }
// void _hite(vel_t va, vel_t vb) {
//     // Serial.printf("apertando=%d %d\t", va, vb);
//     turbo = (va > 0);
// }
