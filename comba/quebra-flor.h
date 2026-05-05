#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Wemos D1 mini (clone) - esp8266 (+ l298n + drv8833)


#define CONTROLE controle_preto_j_verm
#define ARMA_DIGITAL //! isso deveria vir com o controle de alguma forma

#define LED LED_BUILTIN

#define motor_esq_m1 D0
#define motor_esq_m2 D5
#define motor_dir_m1 D6
#define motor_dir_m2 D7

#define motor_arma_m1 D1
#define motor_arma_m2 D2
