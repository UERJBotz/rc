#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Wemos D1 mini (clone) - esp8266 (+ l298n)
//! chamar de abelhas em vez de soldados (brincadeira com a vespa da robocore)

#define CONTROLE controle_branco

#define LED LED_BUILTIN

#define motor_esq_m1 D0
#define motor_esq_m2 D5
#define motor_dir_m1 D6
#define motor_dir_m2 D7
