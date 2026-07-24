#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Wemos D1 mini (clone) - esp8266 (+ l298n)
//! chamar de abelhas em vez de soldados (brincadeira com a vespa da robocore)

#define CONTROLE controle_branco

#define LED LED_BUILTIN

#define motor_esq_m1 D1
#define motor_esq_m2 D2
#define motor_dir_m1 D3
#define motor_dir_m2 D4
