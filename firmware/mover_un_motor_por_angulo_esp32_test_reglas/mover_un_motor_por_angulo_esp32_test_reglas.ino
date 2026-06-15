#include <SCServo.h>

static const long DEBUG_BAUD = 115200;
static const long SERVO_BAUD = 1000000;
static const int RX_PIN = 16;
static const int TX_PIN = 17;

static const uint16_t MOVE_SPEED = 450;
static const uint8_t MOVE_ACC = 45;

static const int MIN_ID = 1;
static const int MAX_ID = 6;
static const unsigned long AUTO_POS_INTERVAL_MS = 200;
static const unsigned long MOVE_COOLDOWN_MS = 1000;
static const unsigned long POST_MOVE_REPORT_MS = 1500;

static const float GEOM_H_BASE = 100.0f;
static const float GEOM_W_BASE = 35.0f;
static const float GEOM_L1 = 113.0f;
static const float GEOM_OFFSET_M3 = 27.0f;
static const float GEOM_L2 = 137.0f;
static const float GEOM_L3 = 60.0f;
static const float GEOM_GRIPPER_LEN = 100.0f;
static const float GEOM_GRIPPER_WIDTH = 65.0f;
static const float FLOOR_TOUCH_THRESHOLD = 5.0f;
static const float FLOOR_RISK_CLEARANCE = 30.0f;

static const float OFFSET_SIGN = 1.0f;
static const bool MIRROR_X = true;
static const float M2_SIGN = 1.0f;
static const float M3_SIGN = 1.0f;
static const float M4_SIGN = 1.0f;

HardwareSerial ServoSerial(2);
SMS_STS servoBus;
String inputLine;
unsigned long lastAutoPosMs = 0;
unsigned long lastMoveCommandMs = 0;
bool pendingMoveReport = false;
uint8_t pendingMoveReportId = 0;
unsigned long pendingMoveReportAtMs = 0;

struct Point2 {
  float x;
  float y;
};

long degToPos(float deg) {
  return lround((deg / 360.0f) * 4095.0f);
}

float degToRad(float deg) {
  return deg * PI / 180.0f;
}

float posToDeg(int pos) {
  if (pos < 0) return -1.0f;
  return (pos * 360.0f) / 4095.0f;
}

bool servoPresent(uint8_t id) {
  return servoBus.Ping(id) != -1;
}

Point2 addPt(Point2 a, Point2 b) {
  Point2 r = {a.x + b.x, a.y + b.y};
  return r;
}

Point2 subPt(Point2 a, Point2 b) {
  Point2 r = {a.x - b.x, a.y - b.y};
  return r;
}

Point2 scalePt(Point2 v, float factor) {
  Point2 r = {v.x * factor, v.y * factor};
  return r;
}

float dotPt(Point2 a, Point2 b) {
  return a.x * b.x + a.y * b.y;
}

float lenPt(Point2 v) {
  return sqrtf(v.x * v.x + v.y * v.y);
}

Point2 vecPt(float length, float angleRad) {
  Point2 r = {length * cosf(angleRad), length * sinf(angleRad)};
  return r;
}

Point2 perpendicularOffset(float length, float angleRad) {
  Point2 r = {
    length * cosf(angleRad + OFFSET_SIGN * PI / 2.0f),
    length * sinf(angleRad + OFFSET_SIGN * PI / 2.0f)
  };
  return r;
}

float pointSegmentDistance(Point2 p, Point2 a, Point2 b) {
  Point2 ab = subPt(b, a);
  Point2 ap = subPt(p, a);
  float abLen2 = dotPt(ab, ab);
  if (abLen2 <= 1e-9f) {
    return lenPt(subPt(p, a));
  }
  float t = dotPt(ap, ab) / abLen2;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  Point2 proj = addPt(a, scalePt(ab, t));
  return lenPt(subPt(p, proj));
}

float orient(Point2 a, Point2 b, Point2 c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment(Point2 a, Point2 b, Point2 p) {
  return (
    min(a.x, b.x) <= p.x && p.x <= max(a.x, b.x) &&
    min(a.y, b.y) <= p.y && p.y <= max(a.y, b.y)
  );
}

bool segmentsIntersect(Point2 a, Point2 b, Point2 c, Point2 d) {
  float o1 = orient(a, b, c);
  float o2 = orient(a, b, d);
  float o3 = orient(c, d, a);
  float o4 = orient(c, d, b);

  if (fabsf(o1) < 1e-6f && onSegment(a, b, c)) return true;
  if (fabsf(o2) < 1e-6f && onSegment(a, b, d)) return true;
  if (fabsf(o3) < 1e-6f && onSegment(c, d, a)) return true;
  if (fabsf(o4) < 1e-6f && onSegment(c, d, b)) return true;
  return ((o1 > 0) != (o2 > 0)) && ((o3 > 0) != (o4 > 0));
}

bool pointInRect(Point2 p, float xMin, float xMax, float yMin, float yMax) {
  return xMin <= p.x && p.x <= xMax && yMin <= p.y && p.y <= yMax;
}

void computePosePoints(float m2Deg, float m3Deg, float m4Deg, Point2 &p0, Point2 &p1, Point2 &p1o, Point2 &p2, Point2 &p3) {
  float a1 = degToRad((m2Deg - 180.0f) * M2_SIGN + 90.0f);
  float a2 = degToRad((m3Deg - 90.0f) * M3_SIGN);
  float a3 = degToRad((m4Deg - 180.0f) * M4_SIGN);

  p0 = {0.0f, GEOM_H_BASE};
  p1 = addPt(p0, vecPt(GEOM_L1, a1));
  p1o = addPt(p1, perpendicularOffset(GEOM_OFFSET_M3, a1));
  p2 = addPt(p1o, vecPt(GEOM_L2, a1 + a2));
  p3 = addPt(p2, vecPt(GEOM_L3, a1 + a2 + a3));

  if (MIRROR_X) {
    p0.x = -p0.x;
    p1.x = -p1.x;
    p1o.x = -p1o.x;
    p2.x = -p2.x;
    p3.x = -p3.x;
  }
}

void computeGripperCorners(float m2Deg, float m3Deg, float m4Deg, Point2 corners[4], Point2 &p0, Point2 &p1, Point2 &p1o, Point2 &p2, Point2 &p3) {
  computePosePoints(m2Deg, m3Deg, m4Deg, p0, p1, p1o, p2, p3);
  Point2 dir = subPt(p3, p2);
  float dirLen = lenPt(dir);
  Point2 u = {1.0f, 0.0f};
  if (dirLen > 1e-6f) {
    u = {dir.x / dirLen, dir.y / dirLen};
  }
  Point2 n = {-u.y, u.x};
  float halfW = GEOM_GRIPPER_WIDTH / 2.0f;

  corners[0] = addPt(p3, scalePt(n, -halfW));
  corners[1] = addPt(p3, scalePt(n, halfW));
  corners[2] = addPt(corners[1], scalePt(u, GEOM_GRIPPER_LEN));
  corners[3] = addPt(corners[0], scalePt(u, GEOM_GRIPPER_LEN));
}

bool collidesWithBase(Point2 corners[4]) {
  float xMin = -GEOM_W_BASE / 2.0f;
  float xMax = GEOM_W_BASE / 2.0f;
  float yMin = 0.0f;
  float yMax = GEOM_H_BASE;
  for (int i = 0; i < 4; i++) {
    if (pointInRect(corners[i], xMin, xMax, yMin, yMax)) return true;
  }

  Point2 rectEdgesA[4] = {
    {xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}
  };
  Point2 rectEdgesB[4] = {
    {xMax, yMin}, {xMax, yMax}, {xMin, yMax}, {xMin, yMin}
  };

  for (int i = 0; i < 4; i++) {
    Point2 a = corners[i];
    Point2 b = corners[(i + 1) % 4];
    for (int j = 0; j < 4; j++) {
      if (segmentsIntersect(a, b, rectEdgesA[j], rectEdgesB[j])) return true;
    }
  }
  return false;
}

bool collidesWithLink(Point2 corners[4], Point2 a, Point2 b, float thickness) {
  float radius = thickness / 2.0f;
  for (int i = 0; i < 4; i++) {
    if (pointSegmentDistance(corners[i], a, b) <= radius) return true;
  }
  for (int i = 0; i < 4; i++) {
    Point2 e0 = corners[i];
    Point2 e1 = corners[(i + 1) % 4];
    if (segmentsIntersect(e0, e1, a, b)) return true;
    if (pointSegmentDistance(e0, a, b) <= radius) return true;
    if (pointSegmentDistance(e1, a, b) <= radius) return true;
  }
  return false;
}

bool classifyPoseInvalid(float m2Deg, float m3Deg, float m4Deg, String &reason) {
  Point2 corners[4];
  Point2 p0, p1, p1o, p2, p3;
  computeGripperCorners(m2Deg, m3Deg, m4Deg, corners, p0, p1, p1o, p2, p3);

  float minY = corners[0].y;
  for (int i = 1; i < 4; i++) {
    if (corners[i].y < minY) minY = corners[i].y;
  }

  if (minY <= FLOOR_TOUCH_THRESHOLD) {
    reason = "PISO";
    return true;
  }
  if (collidesWithBase(corners)) {
    reason = "COLISION_BASE";
    return true;
  }
  if (collidesWithLink(corners, p0, p1, 25.0f)) {
    reason = "COLISION_L1";
    return true;
  }
  if (collidesWithLink(corners, p1o, p2, 25.0f)) {
    reason = "COLISION_L2";
    return true;
  }
  if (minY <= FLOOR_RISK_CLEARANCE) {
    reason = "RIESGO_PISO";
    return true;
  }

  reason = "OK";
  return false;
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
  Serial.println(F("  ID 4:    75..260 con reglas dinamicas"));
  Serial.println(F("  ID 5:    0..340"));
  Serial.println(F("  ID 6:    0..140 logico -> 10..150 real"));
  Serial.println(F("Reglas dinamicas de test para M4:"));
  Serial.println(F("  Si M2<=100 y M3<=110 -> M4<=130"));
  Serial.println(F("  Si M2<=120 y M3<=100 -> M4<=120"));
  Serial.println(F("  Si M2>=240 y M3>=140 -> M4<=200"));
  Serial.println(F("  Si M2>=260 -> M4<=185"));
  Serial.println(F("  Si M3>=250 -> M4<=230"));
  Serial.println(F("  Si M2>=185 y M3>=245 -> M4<=250"));
  Serial.println(F("  Si M2>=200 y M3>=255 -> M4<=245"));
  Serial.println(F("Ejemplos:"));
  Serial.println(F("  1 180"));
  Serial.println(F("  2 90"));
  Serial.println(F("  4 150"));
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

bool getCurrentLogicalDegrees(uint8_t id, float &degreesOut) {
  if (!servoPresent(id)) {
    return false;
  }

  int pos = servoBus.ReadPos(id);
  if (pos < 0) {
    return false;
  }

  float deg = posToDeg(pos);
  if (id == 6) {
    deg -= 10.0f;
  }
  degreesOut = deg;
  return true;
}

bool getCurrentPoseDegrees(float &m2, float &m3, float &m4) {
  return getCurrentLogicalDegrees(2, m2) &&
         getCurrentLogicalDegrees(3, m3) &&
         getCurrentLogicalDegrees(4, m4);
}

float getDynamicM4Max(float m2, float m3) {
  float maxM4 = 260.0f;

  if (m2 <= 100.0f && m3 <= 110.0f) {
    maxM4 = min(maxM4, 130.0f);
  }
  if (m2 <= 120.0f && m3 <= 100.0f) {
    maxM4 = min(maxM4, 120.0f);
  }
  if (m2 >= 240.0f && m3 >= 140.0f) {
    maxM4 = min(maxM4, 200.0f);
  }
  if (m2 >= 260.0f) {
    maxM4 = min(maxM4, 185.0f);
  }
  if (m3 >= 250.0f) {
    maxM4 = min(maxM4, 230.0f);
  }
  if (m2 >= 185.0f && m3 >= 245.0f) {
    maxM4 = min(maxM4, 250.0f);
  }
  if (m2 >= 200.0f && m3 >= 255.0f) {
    maxM4 = min(maxM4, 245.0f);
  }

  return maxM4;
}

bool validateDynamicRules(uint8_t id, float requestedDegrees) {
  float m2 = 0.0f;
  float m3 = 0.0f;
  float m4 = 0.0f;
  if (!getCurrentPoseDegrees(m2, m3, m4)) {
    Serial.println(F("No se pudo leer M2/M3/M4 para validar la pose."));
    return false;
  }

  float targetM2 = m2;
  float targetM3 = m3;
  float targetM4 = m4;
  if (id == 2) targetM2 = requestedDegrees;
  if (id == 3) targetM3 = requestedDegrees;
  if (id == 4) targetM4 = requestedDegrees;

  if (id == 4) {
    float dynamicMax = getDynamicM4Max(m2, m3);
    if (requestedDegrees > dynamicMax) {
      Serial.print(F("Bloqueado: con M2="));
      Serial.print(m2, 1);
      Serial.print(F(" y M3="));
      Serial.print(m3, 1);
      Serial.print(F(", M4 max permitido es "));
      Serial.println(dynamicMax, 1);
      return false;
    }
  }

  if (id == 2 || id == 3 || id == 4) {
    String reason;
    if (classifyPoseInvalid(targetM2, targetM3, targetM4, reason)) {
      Serial.print(F("Bloqueado por simulacion: "));
      Serial.print(reason);
      Serial.print(F(" con M2="));
      Serial.print(targetM2, 1);
      Serial.print(F(" M3="));
      Serial.print(targetM3, 1);
      Serial.print(F(" M4="));
      Serial.println(targetM4, 1);
      return false;
    }
  }

  return true;
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

void processPendingMoveReport() {
  if (!pendingMoveReport) {
    return;
  }

  if (millis() < pendingMoveReportAtMs) {
    return;
  }

  pendingMoveReport = false;
  readOneServo(pendingMoveReportId);
  printDivider();
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

  if (!validateDynamicRules(id, degrees)) {
    printDivider();
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
  pendingMoveReport = true;
  pendingMoveReportId = id;
  pendingMoveReportAtMs = now + POST_MOVE_REPORT_MS;
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

  ServoSerial.begin(SERVO_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  servoBus.pSerial = &ServoSerial;

  printDivider();
  Serial.println(F("Control de un motor por vez - TEST REGLAS"));
  Serial.println(F("Speed fijo = 450"));
  Serial.println(F("Acc fijo = 45"));
  Serial.println(F("Un solo motor por comando."));
  Serial.println(F("Lectura automatica de posiciones cada 0.2s."));
  printHelp();
}

void loop() {
  autoPrintPositionsIfNeeded();
  processPendingMoveReport();

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
