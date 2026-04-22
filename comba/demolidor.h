#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

#define CONTROLE controle_roxo_j_verm

#define LED LED_BUILTIN

#define motor_esq_m1 D0
#define motor_esq_m2 D5
#define motor_dir_m1 D6
#define motor_dir_m2 D7
