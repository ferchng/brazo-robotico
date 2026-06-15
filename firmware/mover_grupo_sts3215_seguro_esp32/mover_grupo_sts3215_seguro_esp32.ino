/*
  mover_grupo_sts3215_seguro_esp32.ino

  Objetivo:
  - Mover de forma muy conservadora servos STS3215 ya configurados.
  - Solo mueve UN motor por vez.
  - Permite elegir grupo 1..3 o 4..6 con un booleano.

  Seguridad:
  - Fuente externa de 12V obligatoria.
  - Con 3A, usar solo movimientos suaves y sin carga.
  - Este sketch nunca mueve mas de 1 motor simultaneamente.
  - Si ves caida de tension, ruido raro o la fuente entra en limite, apagar.
*/

#include <SCServo.h>

static const bool TEST_SECOND_GROUP = false;  // false = IDs 1..3, true = IDs 4..6

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

// Movimiento conservador
static const int DELTA_POS = 80;        // paso chico sobre 4096 cuentas/vuelta
static const uint16_t MOVE_SPEED = 120; // bajo
static const uint8_t MOVE_ACC = 10;     // bajo
static const unsigned long WAIT_AFTER_MOVE_MS = 1800;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

bool servoPresent(uint8_t id) {
  return servoBus.Ping(id) != -1;
}

int readValidPos(uint8_t id) {
  int pos = servoBus.ReadPos(id);
  return pos;
}

void moveOneServoSafely(uint8_t id) {
  if (!servoPresent(id)) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": sin respuesta, se omite."));
    return;
  }

  int startPos = readValidPos(id);
  if (startPos < 0) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": posicion invalida, se omite."));
    return;
  }

  int targetPos = startPos + DELTA_POS;
  if (targetPos > 4095) {
    targetPos = startPos - DELTA_POS;
  }
  if (targetPos < 0) {
    targetPos = startPos;
  }

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": inicio="));
  Serial.print(startPos);
  Serial.print(F(" -> objetivo="));
  Serial.println(targetPos);

  servoBus.WritePosEx(id, targetPos, MOVE_SPEED, MOVE_ACC);
  delay(WAIT_AFTER_MOVE_MS);

  int posAfter = readValidPos(id);
  int voltage = servoBus.ReadVoltage(id);
  int temp = servoBus.ReadTemper(id);

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": posicion actual="));
  Serial.print(posAfter);
  Serial.print(F(" | V="));
  Serial.print(voltage);
  Serial.print(F(" | T="));
  Serial.println(temp);

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.println(F(": regreso a posicion inicial"));
  servoBus.WritePosEx(id, startPos, MOVE_SPEED, MOVE_ACC);
  delay(WAIT_AFTER_MOVE_MS);

  int finalPos = readValidPos(id);
  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": posicion final="));
  Serial.println(finalPos);
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printDivider();
  Serial.println(F("Movimiento seguro STS3215"));
  Serial.println(F("Un motor por vez, paso chico, velocidad baja."));
  Serial.print(F("Grupo: "));
  Serial.println(TEST_SECOND_GROUP ? F("IDs 4..6") : F("IDs 1..3"));
  printDivider();
}

void loop() {
  uint8_t startId = TEST_SECOND_GROUP ? 4 : 1;
  uint8_t endId = TEST_SECOND_GROUP ? 6 : 3;

  for (uint8_t id = startId; id <= endId; id++) {
    moveOneServoSafely(id);
    printDivider();
    delay(1200);
  }

  Serial.println(F("Ciclo completo. Esperando 6 segundos."));
  printDivider();
  delay(6000);
}
