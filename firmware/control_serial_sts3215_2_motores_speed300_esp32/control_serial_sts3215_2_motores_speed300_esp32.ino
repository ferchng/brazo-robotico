#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

static const uint16_t MOVE_SPEED = 300;
static const uint8_t MOVE_ACC = 30;
static const int TOGGLE_STEP = 120;
static const unsigned long TOGGLE_INTERVAL_MS = 120;

static const int MIN_DEG = 0;
static const int MAX_DEG = 360;
static const int MIN_ID = 1;
static const int MAX_ID = 6;
static const unsigned long WAIT_AFTER_MOVE_MS = 1700;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;
bool toggleActive[7] = {false, false, false, false, false, false, false};
int toggleTargetPos[7] = {0, 0, 0, 0, 0, 0, 0};
unsigned long lastToggleStepMs[7] = {0, 0, 0, 0, 0, 0, 0};

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
  Serial.println(F("  toggle <id>"));
  Serial.println(F("  help"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  1 180 ; 2 90"));
  Serial.println(F("  read 3"));
  Serial.println(F("  toggle 1"));
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
  for (uint8_t id = 1; id <= 6; id++) {
    readOneServo(id);
    delay(120);
  }
  printDivider();
}

void stopToggle(uint8_t id) {
  if (id < MIN_ID || id > MAX_ID) {
    return;
  }

  toggleActive[id] = false;
  Serial.print(F("Toggle detenido para ID "));
  Serial.println(id);
}

void startToggle(uint8_t id) {
  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return;
  }

  if (!servoPresent(id)) {
    Serial.print(F("ID "));
    Serial.print(id);
    Serial.println(F(": sin respuesta"));
    return;
  }

  int currentPos = servoBus.ReadPos(id);
  if (currentPos < 0) {
    Serial.println(F("No se pudo leer la posicion actual."));
    return;
  }

  toggleTargetPos[id] = currentPos;
  lastToggleStepMs[id] = millis();
  toggleActive[id] = true;

  Serial.print(F("Toggle iniciado para ID "));
  Serial.print(id);
  Serial.println(F(". Envia el mismo comando para detenerlo."));
}

void handleToggleCommand(uint8_t id) {
  if (id < MIN_ID || id > MAX_ID) {
    Serial.println(F("ID invalido. Usa 1..6."));
    return;
  }

  if (toggleActive[id]) {
    stopToggle(id);
  } else {
    startToggle(id);
  }
  printDivider();
}

void serviceToggleMotion() {
  unsigned long now = millis();

  for (uint8_t id = MIN_ID; id <= MAX_ID; id++) {
    if (!toggleActive[id]) {
      continue;
    }

    if (now - lastToggleStepMs[id] < TOGGLE_INTERVAL_MS) {
      continue;
    }

    if (!servoPresent(id)) {
      toggleActive[id] = false;
      Serial.print(F("Toggle cancelado para ID "));
      Serial.print(id);
      Serial.println(F(" por perdida de respuesta."));
      continue;
    }

    toggleTargetPos[id] += TOGGLE_STEP;
    while (toggleTargetPos[id] > 4095) {
      toggleTargetPos[id] -= 4096;
    }

    servoBus.WritePosEx(id, toggleTargetPos[id], MOVE_SPEED, MOVE_ACC);
    lastToggleStepMs[id] = now;
  }
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
    servoBus.WritePosEx(moves[i].id, (int)targetPos, MOVE_SPEED, MOVE_ACC);
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

  if (line.startsWith("read ")) {
    int id = line.substring(5).toInt();
    readOneServo((uint8_t)id);
    printDivider();
    return;
  }

  if (line.startsWith("toggle ")) {
    int id = line.substring(7).toInt();
    handleToggleCommand((uint8_t)id);
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
  Serial.println(F("Speed fijo = 300"));
  Serial.println(F("Acc fijo = 30"));
  printHelp();
}

void loop() {
  serviceToggleMotion();

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
