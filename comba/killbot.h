#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Nologo ESP32C3 Super Mini


#define CONTROLE controle_preto_j_verm

#define LED LED_BUILTIN

#define motor_esq_m1 5
#define motor_esq_m2 6
#define motor_dir_m1 9
#define motor_dir_m2 10
