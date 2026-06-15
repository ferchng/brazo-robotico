#include <SCServo.h>

// =========================
// Configuracion del usuario
// =========================

// Edita estos valores antes de subir el sketch.
static const uint8_t CURRENT_ID = 1;   // ID actual del servo conectado
static const uint8_t TARGET_ID  = 6;   // Nuevo ID deseado

// Velocidad del monitor serie del PC.
static const long DEBUG_BAUD = 115200;

// Baud tipico de STS3215. Si tu servo no responde, revisa este valor.
static const long SERVO_BAUD = 1000000;

// =========================
// Seleccion de puerto serie
// =========================
//
// Ajusta esta seccion segun tu placa.
// Ejemplos comunes:
// - ESP32: usar Serial2 con pines RX/TX configurables.
// - Mega2560: usar Serial1.
// - Otras placas: adaptar a su UART disponible.

#if defined(ARDUINO_ARCH_ESP32)
HardwareSerial ServoSerial(2);
static const int SERVO_RX_PIN = 16;
static const int SERVO_TX_PIN = 17;
#define SERVO_PORT ServoSerial
#elif defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
#define SERVO_PORT Serial1
#else
#error "Esta placa no esta preconfigurada. Usa ESP32 o Mega, o adapta SERVO_PORT."
#endif

SMS_STS servoBus;

void printDivider() {
  Serial.println(F("--------------------------------------------------"));
}

bool pingServo(uint8_t id) {
  int response = servoBus.Ping(id);
  return response != -1;
}

void setupServoPort() {
#if defined(ARDUINO_ARCH_ESP32)
  SERVO_PORT.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
#else
  SERVO_PORT.begin(SERVO_BAUD);
#endif

  servoBus.pSerial = &SERVO_PORT;
}

void printIntro() {
  printDivider();
  Serial.println(F("Asignacion de ID para Feetech STS3215"));
  Serial.println(F("Modo seguro: cambiar ID sin mover el servo."));
  printDivider();
  Serial.print(F("CURRENT_ID = "));
  Serial.println(CURRENT_ID);
  Serial.print(F("TARGET_ID  = "));
  Serial.println(TARGET_ID);
  Serial.print(F("SERVO_BAUD = "));
  Serial.println(SERVO_BAUD);
  printDivider();
}

void failAndStop(const __FlashStringHelper* message) {
  Serial.println(message);
  Serial.println(F("Proceso cancelado."));
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(1500);

  setupServoPort();
  printIntro();

  if (CURRENT_ID < 1 || CURRENT_ID > 253) {
    failAndStop(F("ERROR: CURRENT_ID fuera de rango valido (1..253)."));
  }

  if (TARGET_ID < 1 || TARGET_ID > 253) {
    failAndStop(F("ERROR: TARGET_ID fuera de rango valido (1..253)."));
  }

  if (CURRENT_ID == TARGET_ID) {
    Serial.println(F("Aviso: CURRENT_ID y TARGET_ID son iguales."));
    Serial.println(F("No hay nada que cambiar. Solo se hara verificacion."));
    if (pingServo(CURRENT_ID)) {
      Serial.println(F("Servo detectado correctamente."));
    } else {
      Serial.println(F("No se detecto servo con ese ID."));
    }
    while (true) {
      delay(1000);
    }
  }

  Serial.println(F("Paso 1: verificando que exista el servo con CURRENT_ID..."));
  if (!pingServo(CURRENT_ID)) {
    failAndStop(F("ERROR: no responde ningun servo con CURRENT_ID."));
  }
  Serial.println(F("OK: servo actual detectado."));

  Serial.println(F("Paso 2: verificando que TARGET_ID este libre..."));
  if (pingServo(TARGET_ID)) {
    failAndStop(F("ERROR: TARGET_ID ya responde en el bus. No se cambiara nada."));
  }
  Serial.println(F("OK: TARGET_ID libre."));

  Serial.println(F("Paso 3: desbloqueando EEPROM..."));
  servoBus.unLockEprom(CURRENT_ID);
  delay(20);

  Serial.println(F("Paso 4: escribiendo nuevo ID..."));
  servoBus.writeByte(CURRENT_ID, SMS_STS_ID, TARGET_ID);
  delay(100);

  Serial.println(F("Paso 5: bloqueando EEPROM con el nuevo ID..."));
  servoBus.LockEprom(TARGET_ID);
  delay(100);

  Serial.println(F("Paso 6: verificando respuesta con el nuevo ID..."));
  if (!pingServo(TARGET_ID)) {
    failAndStop(F("ERROR: no hubo respuesta con el nuevo ID. Revisar cableado/baud."));
  }

  Serial.println(F("OK: cambio de ID completado."));
  Serial.print(F("ID anterior: "));
  Serial.println(CURRENT_ID);
  Serial.print(F("ID nuevo: "));
  Serial.println(TARGET_ID);
  printDivider();
  Serial.println(F("Apaga la alimentacion antes de cambiar al siguiente servo."));
}

void loop() {
  delay(1000);
}
