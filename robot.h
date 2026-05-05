typedef int16_t vel_t;

void     robot_setup(void);
void     move(vel_t, vel_t); //-VEL_MAX a VEL_MAX
void     hite(vel_t);        //-VEL_MAX a VEL_MAX
void     bipe(int);          //millis
uint32_t batt(void);         //mV

#if !defined(VEL_MAX)
    #error "constante VEL_MAX precisa estar definida"
#endif
#if VEL_MAX > INT16_MAX
    #error "constante VEL_MAX precisa ter um valor são"
#endif

#if defined(VESPA)
    #include <RoboCore_Vespa.h>

    VespaLED     led;
    VespaButton  button;
    VespaBattery vbat;
    VespaMotors  motors;

    void robot_setup(void){}
    void move(vel_t esq, vel_t dir) {
        motors.turn(
            map(esq, -VEL_MAX,VEL_MAX, -100,100),
            map(dir, -VEL_MAX,VEL_MAX, -100,100)
        );
    }
    void hite(vel_t){}
    void bipe(int dt) {
        move(20, 20); delay(dt);
        move(0, 0);
    }
    uint32_t batt() {
        return vbat.readVoltage(); //mV
    }
#elif defined(COMBA)
    #if LED_BUILTIN && !defined(LED)
        #define LED LED_BUILTIN
    #endif
    #if defined(ESC_ARMA)
        #if defined(motor_arma_m1) || defined(motor_arma_m2)
            #error "escolha entre esc e ponte h para a arma"
        #endif

        #include <Servo.h>
        Servo arma;
    #endif

    void robot_setup(void) {
      #ifdef LED
        pinMode(LED, OUTPUT);
      #endif
      #ifdef BAT
        pinMode(BAT, INPUT);
      #endif

      #ifdef ESC_ARMA
        arma.attach(ESC_ARMA);
      #endif

      #if defined(motor_arma_m1) && defined(motor_arma_m2)
        pinMode(motor_arma_m1, OUTPUT);
        pinMode(motor_arma_m2, OUTPUT);
      #elif defined(motor_arma_m1) || defined(motor_arma_m2)
        #error "defina ambos os pinos do motor da arma"
      #endif

        pinMode(motor_esq_m1, OUTPUT);
        pinMode(motor_dir_m2, OUTPUT);
        pinMode(motor_esq_m1, OUTPUT);
        pinMode(motor_dir_m2, OUTPUT);
    }
    void motor(uint8_t m1, uint8_t m2, int16_t vel) {
        if (vel < 0) {
            analogWrite(m1, abs(vel));
            analogWrite(m2, 0);
        } else {
            analogWrite(m1, 0);
            analogWrite(m2, vel);
        }
    }
    void move(vel_t esq, vel_t dir) {
        esq = constrain(
            map(esq, -VEL_MAX, VEL_MAX, -1023, 1023),
            -1023, 1023
        );
        dir = constrain(
            map(dir, -VEL_MAX, VEL_MAX, -1023, 1023),
            -1023, 1023
        );

        motor(motor_esq_m1, motor_esq_m2, esq);
        motor(motor_dir_m1, motor_dir_m2, dir);
    }

    void hite(vel_t vel) {
      #ifdef ARMA_DIGITAL
        vel = map(vel, -VEL_MAX, VEL_MAX, 0, VEL_MAX);
      #endif //! ver jeito melhor de lidar com isso

      #ifdef ESC_ARMA
        #warning "esc experimental!"
        arma.write(map(vel, -VEL_MAX, VEL_MAX, 0, 180));
      #endif

      #if defined(motor_arma_m1) && defined(motor_arma_m2)
        motor(motor_arma_m1, motor_arma_m2, vel);
      #endif
    }
    void bipe(int dt) { //! números do bipe
        move(20, 20); delay(dt);
        move(0, 0);
    }
    uint32_t batt() {
      #ifdef BAT
        #warning "leitura da bateria deve tar errada!"
        return analogRead(BAT); //! mV
      #else
        #warning "leitura da bateria hardcoded!"
        return 8000; //mV
      #endif
    }
#else
    #error "robôs por enquanto só vespa ou comba"
#endif

