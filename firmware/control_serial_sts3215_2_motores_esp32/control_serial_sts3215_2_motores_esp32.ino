/*
  control_serial_sts3215_2_motores_esp32.ino

  Objetivo:
  - Controlar servos STS3215 desde el Monitor Serie.
  - Permite mover 1 servo o 2 servos simultaneamente.
  - Nunca acepta mas de 2 motores por comando.

  Comandos:
  - "<id> <grados>"
      Ejemplo: 1 180
  - "<id1> <grados1> ; <id2> <grados2>"
      Ejemplo: 1 180 ; 2 90
  - "read <id>"
  - "scan"
  - "help"

  Seguridad:
  - Maximo 2 servos simultaneos.
  - Con fuente de 12V 3A: usar sin carga o carga muy baja.
  - Si hay caida de tension, ruido raro o perdida de respuesta, detener prueba.
*/

#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

// Ajusta estos valores si quieres mas velocidad.
// Sube de a poco para no castigar la fuente ni la mecanica.
static uint16_t moveSpeed = 300;
static uint8_t moveAcc = 30;

static const int MIN_DEG = 0;
static const int MAX_DEG = 360;
static const int MIN_ID = 1;
static const int MAX_ID = 6;
static const unsigned long WAIT_AFTER_MOVE_MS = 1700;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;

struct MoveCommand {
  uint8_t id;
  float degrees;
};

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
  Serial.println(F("Comandos:"));
  Serial.println(F("  <id> <grados>"));
  Serial.println(F("  <id1> <grados1> ; <id2> <grados2>"));
  Serial.println(F("  read <id>"));
  Serial.println(F("  scan"));
  Serial.println(F("  speed <valor>    -> cambia velocidad global"));
  Serial.println(F("  acc <valor>      -> cambia aceleracion global"));
  Serial.println(F("  help"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  1 180 ; 2 90"));
  Serial.println(F("  read 3"));
  Serial.println(F("  speed 400"));
  Serial.println(F("  acc 40"));
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

bool parseSingleMove(String part, MoveCommand &cmd) {
  part.trim();
  int spaceIndex = part.indexOf(' ');
  if (spaceIndex == -1) {
    return false;
  }

  String idPart = part.substring(0, spaceIndex);
  String degPart = part.substring(spaceIndex + 1);
  idPart.trim();
  degPart.trim();

  int id = idPart.toInt();
  float degrees = degPart.toFloat();

  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return false;
  }

  if (degrees < MIN_DEG || degrees > MAX_DEG) {
    Serial.println(F("Grados fuera de rango. Usa 0..360."));
    return false;
  }

  if (!servoPresent((uint8_t)id)) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": sin respuesta"));
    return false;
  }

  cmd.id = (uint8_t)id;
  cmd.degrees = degrees;
  return true;
}

void executeMoves(const MoveCommand *moves, int count) {
  if (count < 1 || count > 2) {
    Serial.println(F("Solo se permiten 1 o 2 motores por comando."));
    return;
  }

  if (count == 2 && moves[0].id == moves[1].id) {
    Serial.println(F("No repitas el mismo ID en el mismo comando."));
    return;
  }

  for (int i = 0; i < count; i++) {
    long targetPos = degToPos(moves[i].degrees);
    Serial.print(F("Preparado ID "));
    Serial.print(moves[i].id);
    Serial.print(F(" -> "));
    Serial.print(moves[i].degrees, 1);
    Serial.print(F(" deg -> pos "));
    Serial.println(targetPos);
  }

  Serial.println(count == 1 ? F("Moviendo 1 servo...") : F("Moviendo 2 servos simultaneamente..."));

  for (int i = 0; i < count; i++) {
    long targetPos = degToPos(moves[i].degrees);
    servoBus.WritePosEx(moves[i].id, (int)targetPos, moveSpeed, moveAcc);
    delay(15);
  }

  delay(WAIT_AFTER_MOVE_MS);

  for (int i = 0; i < count; i++) {
    readOneServo(moves[i].id);
  }
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

  if (line.startsWith("speed ")) {
    int value = line.substring(6).toInt();
    if (value < 0 || value > 4095) {
      Serial.println(F("Velocidad invalida. Usa 0..4095."));
    } else {
      moveSpeed = (uint16_t)value;
      Serial.print(F("Velocidad actual = "));
      Serial.println(moveSpeed);
      printDivider();
    }
    return;
  }

  if (line.startsWith("acc ")) {
    int value = line.substring(4).toInt();
    if (value < 0 || value > 255) {
      Serial.println(F("Aceleracion invalida. Usa 0..255."));
    } else {
      moveAcc = (uint8_t)value;
      Serial.print(F("Aceleracion actual = "));
      Serial.println(moveAcc);
      printDivider();
    }
    return;
  }

  if (line.startsWith("read ")) {
    int id = line.substring(5).toInt();
    readOneServo((uint8_t)id);
    printDivider();
    return;
  }

  int separatorIndex = line.indexOf(';');
  if (separatorIndex == -1) {
    MoveCommand oneMove;
    if (!parseSingleMove(line, oneMove)) {
      Serial.println(F("Comando invalido. Usa 'help'."));
      return;
    }
    executeMoves(&oneMove, 1);
    return;
  }

  String firstPart = line.substring(0, separatorIndex);
  String secondPart = line.substring(separatorIndex + 1);

  MoveCommand moves[2];
  if (!parseSingleMove(firstPart, moves[0])) {
    return;
  }
  if (!parseSingleMove(secondPart, moves[1])) {
    return;
  }

  executeMoves(moves, 2);
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printDivider();
  Serial.println(F("Control serie STS3215"));
  Serial.println(F("Maximo 2 motores simultaneos por seguridad."));
  Serial.println(F("Con fuente de 12V 3A: movimientos suaves y sin carga."));
  Serial.print(F("Velocidad inicial = "));
  Serial.println(moveSpeed);
  Serial.print(F("Aceleracion inicial = "));
  Serial.println(moveAcc);
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
