#if defined(COMBA) || defined(VESPA)
    #error "inclua somente 1 robô"
#endif
#define COMBA

// esp32 dev module + placa dos minis
//! é um mini mas com controle normal, tá faltando unificar

#define MA1 18
#define MA2 19
#define MB1 4
#define MB2 23

#define motor_esq_m1 MA1
#define motor_esq_m2 MA2
#define motor_dir_m1 MB1
#define motor_dir_m2 MB2

#define LED 2

#define CONTROLE controle_branco
