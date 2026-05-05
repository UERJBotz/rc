#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// wemos d1 mini (clone) - esp8266


#define CONTROLE controle_branco

#define LED D4

#define motor_esq_m1 D5
#define motor_esq_m2 D6
#define motor_dir_m1 D7
#define motor_dir_m2 D8
