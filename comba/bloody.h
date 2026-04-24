#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// DOIT ESP32 DEVKIT V1 (o indivíduo é com os jumpers rxtx)


#define CONTROLE controle_branco

#define LED LED_BUILTIN

#define motor_esq_m1 32
#define motor_esq_m2 33
#define motor_dir_m1 25
#define motor_dir_m2 26
