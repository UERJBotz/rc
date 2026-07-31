#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Generic ESP8266 Module


#define CONTROLE controle_preto_j_verm

#define D1 5
#define D2 4
#define D3 0
#define D4 2

#define LED LED_BUILTIN

#define motor_esq_m1 D1
#define motor_esq_m2 D2
#define motor_dir_m1 D4
#define motor_dir_m2 D3
