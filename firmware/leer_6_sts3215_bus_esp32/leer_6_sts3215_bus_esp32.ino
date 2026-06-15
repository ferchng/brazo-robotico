#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
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

  printDivider();
  Serial.println(F("Lectura de 6 servos STS3215 en bus"));
  Serial.println(F("Sin movimiento, solo telemetria."));
  Serial.print(F("Baud servo: "));
  Serial.println(SERVO_BAUD);
  printDivider();
}

void loop() {
  for (uint8_t id = 1; id <= 6; id++) {
    printServoData(id);
    delay(150);
  }

  printDivider();
  delay(4000);
}
