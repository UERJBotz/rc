#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA


#define CONTROLE controle_preto_j_verm
#define ARMA_DIGITAL //! isso deveria vir com o controle de alguma forma

#define LED LED_BUILTIN

#define motor_esq_m1 0
#define motor_esq_m2 1
#define motor_dir_m1 2
#define motor_dir_m2 3

