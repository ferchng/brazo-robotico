#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

static const uint16_t MOVE_SPEED = 300;
static const uint8_t MOVE_ACC = 30;

static const int MIN_ID = 1;
static const int MAX_ID = 6;
static const unsigned long WAIT_AFTER_MOVE_MS = 1500;
static const unsigned long AUTO_POS_INTERVAL_MS = 200;
static const unsigned long MOVE_COOLDOWN_MS = 1000;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;
unsigned long lastAutoPosMs = 0;
unsigned long lastMoveCommandMs = 0;

long degToPos(float deg) {
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
  Serial.println(F("  <id> <grados>"));
  Serial.println(F("  read <id>"));
  Serial.println(F("  scan"));
  Serial.println(F("  posall"));
  Serial.println(F("  help"));
  Serial.print(F("Cooldown entre movimientos: "));
  Serial.print(MOVE_COOLDOWN_MS);
  Serial.println(F(" ms"));
  Serial.println(F("Limites configurados:"));
  Serial.println(F("  ID 1:    60..300"));
  Serial.println(F("  ID 2..3: 90..270"));
  Serial.println(F("  ID 4:    75..260"));
  Serial.println(F("  ID 5:    0..340"));
  Serial.println(F("  ID 6:    0..140 logico -> 10..150 real"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  2 90"));
  Serial.println(F("  read 3"));
  Serial.println(F("  posall"));
  printDivider();
}

bool getUserAngleLimits(uint8_t id, float &minDeg, float &maxDeg) {
  switch (id) {
    case 1:
      minDeg = 60.0f;
      maxDeg = 300.0f;
      return true;
    case 2:
    case 3:
      minDeg = 90.0f;
      maxDeg = 270.0f;
      return true;
    case 4:
      minDeg = 75.0f;
      maxDeg = 260.0f;
      return true;
    case 5:
      minDeg = 0.0f;
      maxDeg = 340.0f;
      return true;
    case 6:
      minDeg = 0.0f;
      maxDeg = 140.0f;
      return true;
    default:
      return false;
  }
}

float toRealServoDegrees(uint8_t id, float userDegrees) {
  if (id == 6) {
    return userDegrees + 10.0f;
  }
  return userDegrees;
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

  Serial.print(F("ID "));
  Serial.print(id);
  Serial.print(F(": pos="));
  Serial.print(pos);
  Serial.print(F(" (~"));
  Serial.print(posToDeg(pos), 1);
  Serial.print(F(" deg) | V="));
  Serial.print(voltage);
  Serial.print(F(" | T="));
  Serial.println(temp);
}

void scanAll() {
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    readOneServo(id);
    delay(120);
  }
  printDivider();
}

void printAllPositions() {
  bool first = true;
  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (!first) {
      Serial.print(F(" | "));
    }
    first = false;

    Serial.print(F("M"));
    Serial.print(id);
    Serial.print(F("="));

    if (!servoPresent(id)) {
      Serial.print(F("NA"));
      continue;
    }

    int pos = servoBus.ReadPos(id);
    Serial.print(pos);
    Serial.print(F("("));
    Serial.print(posToDeg(pos), 1);
    Serial.print(F("deg)"));
    delay(40);
  }
  Serial.println();
  printDivider();
}

void autoPrintPositionsIfNeeded() {
  unsigned long now = millis();
  if (now - lastAutoPosMs < AUTO_POS_INTERVAL_MS) {
    return;
  }

  lastAutoPosMs = now;

  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (!servoPresent(id)) {
      continue;
    }

    int pos = servoBus.ReadPos(id);
    Serial.print(F("POS ID "));
    Serial.print(id);
    Serial.print(F(": "));
    Serial.print(pos);
    Serial.print(F(" (~"));
    Serial.print(posToDeg(pos), 1);
    Serial.println(F(" deg)"));
  }
}

void moveServoToDegrees(uint8_t id, float degrees) {
  float minDeg = 0.0f;
  float maxDeg = 0.0f;
  unsigned long now = millis();

  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return;
  }

  if (now - lastMoveCommandMs < MOVE_COOLDOWN_MS) {
    Serial.print(F("Cooldown activo. Espera "));
    Serial.print(MOVE_COOLDOWN_MS - (now - lastMoveCommandMs));
    Serial.println(F(" ms antes del siguiente movimiento."));
    return;
  }

  if (!getUserAngleLimits(id, minDeg, maxDeg)) {
    Serial.println(F("No hay limites definidos para ese ID."));
    return;
  }

  if (degrees < minDeg || degrees > maxDeg) {
    Serial.print(F("Grados fuera de rango para ID "));
    Serial.print(id);
    Serial.print(F(". Usa "));
    Serial.print(minDeg, 1);
    Serial.print(F(".."));
    Serial.println(maxDeg, 1);
    return;
  }

  if (!servoPresent(id)) {
    Serial.println(F("Ese servo no responde."));
    return;
  }

  float realDegrees = toRealServoDegrees(id, degrees);
  long targetPos = degToPos(realDegrees);

  Serial.print(F("Moviendo ID "));
  Serial.print(id);
  Serial.print(F(" a "));
  Serial.print(degrees, 1);
  Serial.print(F(" deg"));
  if (id == 6) {
    Serial.print(F(" (real "));
    Serial.print(realDegrees, 1);
    Serial.print(F(" deg)"));
  }
  Serial.print(F(" -> pos "));
  Serial.println(targetPos);

  lastMoveCommandMs = now;
  servoBus.WritePosEx(id, (int)targetPos, MOVE_SPEED, MOVE_ACC);
  delay(WAIT_AFTER_MOVE_MS);
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

  if (line.equalsIgnoreCase("posall")) {
    printAllPositions();
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
  Serial.println(F("Control de un motor por vez"));
  Serial.println(F("Speed fijo = 300"));
  Serial.println(F("Acc fijo = 30"));
  Serial.println(F("Un solo motor por comando."));
  Serial.println(F("Lectura automatica de posiciones cada 0.2s."));
  printHelp();
}

void loop() {
  autoPrintPositionsIfNeeded();

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
