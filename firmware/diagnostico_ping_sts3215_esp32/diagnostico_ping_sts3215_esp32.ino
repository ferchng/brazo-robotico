/*
  diagnostico_ping_sts3215_esp32.ino

  Objetivo:
  - Verificar si un solo STS3215 responde por UART sin cambiar ID ni moverlo.
  - Pensado para ESP32 + Waveshare Bus Servo Adapter (A) en modo A.

  Cableado esperado:
  - GPIO16 ESP32 -> RX del adaptador
  - GPIO17 ESP32 -> TX del adaptador
  - GND ESP32    -> GND del adaptador
  - Fuente 12V   -> DC+ / DC- del adaptador
  - Un solo servo al puerto D/V/G
  - Jumper del adaptador en A

  Si no responde:
  - Probar con RX/TX invertidos fisicamente una vez.
  - Verificar fuente encendida en 12.0V.
  - Verificar que el servo sea la version 12V.
*/

#include <SCServo.h>

static const int RX_PIN = 16;
static const int TX_PIN = 17;
static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const uint8_t TEST_ID = 1;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  Serial.println();
  Serial.println(F("Diagnostico STS3215"));
  Serial.println(F("Sin cambio de ID, sin movimiento."));
  Serial.print(F("RX pin ESP32: "));
  Serial.println(RX_PIN);
  Serial.print(F("TX pin ESP32: "));
  Serial.println(TX_PIN);
  Serial.print(F("Baud servo: "));
  Serial.println(SERVO_BAUD);
  Serial.print(F("ID probado: "));
  Serial.println(TEST_ID);
  Serial.println(F("Intentando ping cada 2 segundos..."));

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;
}

void loop() {
  int id = servoBus.Ping(TEST_ID);
  if (id != -1) {
    Serial.print(F("OK, servo detectado con ID "));
    Serial.println(id);
  } else {
    Serial.println(F("Sin respuesta. Revisar fuente, GND comun y RX/TX."));
  }
  delay(2000);
}
