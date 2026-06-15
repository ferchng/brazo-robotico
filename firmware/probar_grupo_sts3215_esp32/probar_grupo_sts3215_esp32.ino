/*
  probar_grupo_sts3215_esp32.ino

  Objetivo:
  - Probar servos STS3215 ya configurados con IDs unicos.
  - Leer presencia y telemetria basica sin mover motores.
  - Permite elegir entre probar IDs 1..3 o IDs 4..6 con un booleano.

  Requisitos:
  - ESP32
  - Waveshare Bus Servo Adapter (A) con jumper en A
  - Fuente externa 12V
  - GND comun entre ESP32 y el adaptador
  - Servos conectados al bus

  Cableado usado por este sketch:
  - GPIO16 ESP32 -> RX del adaptador
  - GPIO17 ESP32 -> TX del adaptador
  - GND ESP32    -> GND del adaptador

  Seguridad:
  - Este sketch NO mueve motores.
  - Solo lee estado cada 3 segundos.
  - Si usas varios servos a la vez, vigila corriente y temperatura.
*/

#include <SCServo.h>

// =========================
// Configuracion del usuario
// =========================

// false -> prueba IDs 1,2,3
// true  -> prueba IDs 4,5,6
static const bool TEST_SECOND_GROUP = false;

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;

static const int RX_PIN = 16;
static const int TX_PIN = 17;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

void printHeader() {
  printDivider();
  Serial.println(F("Prueba segura de grupo STS3215"));
  Serial.println(F("Sin movimiento, solo lectura."));
  Serial.print(F("Baud servo: "));
  Serial.println(SERVO_BAUD);
  Serial.print(F("Grupo seleccionado: "));
  if (TEST_SECOND_GROUP) {
    Serial.println(F("IDs 4..6"));
  } else {
    Serial.println(F("IDs 1..3"));
  }
  printDivider();
}

void printServoData(uint8_t id) {
  int ping = servoBus.Ping(id);
  if (ping == -1) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": sin respuesta"));
    return;
  }

  int pos = servoBus.ReadPos(id);
  int voltage = servoBus.ReadVoltage(id);
  int temp = servoBus.ReadTemper(id);
  int speed = servoBus.ReadSpeed(id);
  int load = servoBus.ReadLoad(id);
  int current = servoBus.ReadCurrent(id);
  int moving = servoBus.ReadMove(id);

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": pos="));
  Serial.print(pos);
  Serial.print(F(" | V="));
  Serial.print(voltage);
  Serial.print(F(" | T="));
  Serial.print(temp);
  Serial.print(F(" | speed="));
  Serial.print(speed);
  Serial.print(F(" | load="));
  Serial.print(load);
  Serial.print(F(" | current="));
  Serial.print(current);
  Serial.print(F(" | moving="));
  Serial.println(moving);
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printHeader();
}

void loop() {
  uint8_t startId = TEST_SECOND_GROUP ? 4 : 1;
  uint8_t endId = TEST_SECOND_GROUP ? 6 : 3;

  for (uint8_t id = startId; id <= endId; id++) {
    printServoData(id);
    delay(120);
  }

  printDivider();
  delay(3000);
}
