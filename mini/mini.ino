#include <Arduino.h>

#define VEL_MAX 1023
#define CONTROLE pistola_elton
#include "ultra_t.h" // esse include muda os pinos do robô
#include "_robot.h"  // esse tem uma implementação genérica dos robôs
#include "_comms.h"  // comunicação

#define BAUD_RATE 115200

#define memeql(a,b,sz) (memcmp(a,b,sz) == 0)
#define LEN(arr) (sizeof(arr)/sizeof(*arr))

//! isso é o código da pistola do elton, não é muito legal,
////! acho que devia mudar. no mínimo colocar dentro de um #ifdef

void on_recv(const esp_now_recv_info_t* info, const uint8_t* data, esp_now_len_t len) {
  int gatilhoNorm, rodaNorm;

  struct rc {
      int gatilho;
      int roda;
  } pack_rc;

  #ifdef CONTROLE
    if (!memeql(info->src_addr, CONTROLE, sizeof(CONTROLE))) return;
  #else
    #warning "aceitando conexão de qualquer controle"
  #endif

  memcpy(&pack_rc, data, sizeof(pack_rc));

  // Normalização do gatilho
  if (pack_rc.gatilho < 1800) {
    gatilhoNorm = map(pack_rc.gatilho, 820, 1800, 255, 0);
  } else if (pack_rc.gatilho > 2300) {
    gatilhoNorm = map(pack_rc.gatilho, 2300, 2970, 0, -255);
  } else {
    gatilhoNorm = 0;
  }

  // Normalização da roda
  if (pack_rc.roda < 1800) {
    rodaNorm = map(pack_rc.roda, 150, 1800, 127, 0);
  } else if (pack_rc.roda > 2300) {
    rodaNorm = map(pack_rc.roda, 2300, 4050, 0, -127);
  } else {
    rodaNorm = 0;
  }

  Serial.print("Bruto -> G: ");          Serial.print(pack_rc.gatilho);
  Serial.print(" R: ");                  Serial.print(pack_rc.roda);
  Serial.print(" | Normalizado -> G: "); Serial.print(gatilhoNorm);
  Serial.print(" R: ");                  Serial.println(rodaNorm);

  int y = map(gatilhoNorm, -280, 180, -1023, 1023);
  int z = map(rodaNorm,    -130, 120, -1023, 1023);

  int motorEsq = constrain(-z + y, -1023, 1023);
  int motorDir = constrain( z + y, -1023, 1023);

  const int deadzone = 200; // elton acho que de 200 à 500 é bom

  if (abs(motorEsq) < deadzone &&
      abs(motorDir) < deadzone) {
    Serial.println("MOTORES PARADOS");
    move(0,0);
  } else {
    move(motorEsq, motorDir);
  }
}

void setup() {
    Serial.begin(BAUD_RATE);

    robot_setup();
    espnow_setup(on_recv);
}
void loop() {}
