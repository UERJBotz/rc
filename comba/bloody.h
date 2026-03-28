#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA


#define CONTROLE controle_branco

#define LED LED_BUILTIN

#define D1 5
#define D2 4
#define D3 0
#define D4 2

#define motor_esq_m1 D1
#define motor_esq_m2 D2
#define motor_dir_m1 D3
#define motor_dir_m2 D4
