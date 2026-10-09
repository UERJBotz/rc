#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// Nologo esp32c3 super mini


#define CONTROLE controle_preto_j_verm
#define ARMA_DIGITAL //! isso deveria vir com o controle de alguma forma

#define LED LED_BUILTIN

#define motor_esq_m1 6
#define motor_esq_m2 5
#define motor_dir_m1 8
#define motor_dir_m2 7

#define motor_arma_m1 9
#define motor_arma_m2 10
