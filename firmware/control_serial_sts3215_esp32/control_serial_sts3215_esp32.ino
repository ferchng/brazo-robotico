/*
  control_serial_sts3215_esp32.ino

  Objetivo:
  - Controlar servos STS3215 desde el Monitor Serie con comandos tipo:
      1 180
    que significa:
      mover servo ID 1 a 180 grados

  Seguridad:
  - Solo mueve UN servo por comando.
  - Velocidad y aceleracion conservadoras.
  - No mueve mas de un motor simultaneamente.
  - Con tu fuente de 12V 3A, usar sin carga o con carga muy baja.
  - Si el brazo esta montado mecanicamente, no assumes que 0..360 sea seguro.

  Sintaxis:
  - "<id> <grados>"
    Ejemplos:
      1 180
      2 90
      6 10
  - "read <id>"
    Ejemplo:
      read 3
  - "scan"
    Lee IDs 1..6
  - "help"
    Muestra ayuda

  Cableado:
  - GPIO16 ESP32 -> RX del adaptador
  - GPIO17 ESP32 -> TX del adaptador
  - GND ESP32    -> GND del adaptador
  - Fuente 12V   -> DC+ / DC- del adaptador
  - Servos en bus, jumper del adaptador en A
*/

#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

static const uint16_t MOVE_SPEED = 120;
static const uint8_t MOVE_ACC = 10;

static const int MIN_DEG = 0;
static const int MAX_DEG = 360;
static const int MIN_ID = 1;
static const int MAX_ID = 6;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;

long degToPos(float deg) {
  if (deg < MIN_DEG) deg = MIN_DEG;
  if (deg > MAX_DEG) deg = MAX_DEG;
  return lround((deg / 360.0f) * 4095.0f);
}

float posToDeg(int pos) {
  if (pos < 0) return -1.0f;
  return (pos * 360.0f) / 4095.0f;
}

bool servoPresent(uint8_t id) {
  return servoBus.Ping(id) != -1;
}

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

void printHelp() {
  printDivider();
  Serial.println(F("Comandos disponibles:"));
  Serial.println(F("  <id> <grados>   -> mueve un servo"));
  Serial.println(F("  read <id>       -> lee posicion, voltaje y temperatura"));
  Serial.println(F("  scan            -> escanea IDs 1..6"));
  Serial.println(F("  help            -> muestra esta ayuda"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  2 90"));
  Serial.println(F("  read 4"));
  printDivider();
}

void readOneServo(uint8_t id) {
  if (!servoPresent(id)) {
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

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": pos="));
  Serial.print(pos);
  Serial.print(F(" (~"));
  Serial.print(posToDeg(pos), 1);
  Serial.print(F(" deg) | V="));
  Serial.print(voltage);
  Serial.print(F(" | T="));
  Serial.print(temp);
  Serial.print(F(" | speed="));
  Serial.print(speed);
  Serial.print(F(" | load="));
  Serial.print(load);
  Serial.print(F(" | current="));
  Serial.println(current);
}

void scanAll() {
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    readOneServo(id);
    delay(120);
  }
  printDivider();
}

void moveServoToDegrees(uint8_t id, float degrees) {
  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return;
  }

  if (!servoPresent(id)) {
    Serial.println(F("Ese servo no responde."));
    return;
  }

  if (degrees < MIN_DEG || degrees > MAX_DEG) {
    Serial.println(F("Grados fuera de rango. Usa 0..360."));
    return;
  }

  long targetPos = degToPos(degrees);

  Serial.print(F("Moviendo ID "));
  Serial.print(id);
  Serial.print(F(" a "));
  Serial.print(degrees, 1);
  Serial.print(F(" deg -> pos "));
  Serial.println(targetPos);

  servoBus.WritePosEx(id, (int)targetPos, MOVE_SPEED, MOVE_ACC);
  delay(1500);
  readOneServo(id);
  printDivider();
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("scan")) {
    scanAll();
    return;
  }

  if (line.startsWith("read ")) {
    int id = line.substring(5).toInt();
    readOneServo((uint8_t)id);
    printDivider();
    return;
  }

  int spaceIndex = line.indexOf(' ');
  if (spaceIndex == -1) {
    Serial.println(F("Comando invalido. Usa 'help'."));
    return;
  }

  String idPart = line.substring(0, spaceIndex);
  String degPart = line.substring(spaceIndex + 1);

  int id = idPart.toInt();
  float degrees = degPart.toFloat();

  moveServoToDegrees((uint8_t)id, degrees);
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printDivider();
  Serial.println(F("Control serie STS3215"));
  Serial.println(F("Un motor por comando. Maximo 1 simultaneo."));
  Serial.println(F("Fuente sugerida: 12.0V y carga baja."));
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleLine(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}
