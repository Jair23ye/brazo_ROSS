// Prueba del motor a pasos de la base (NEMA 23 + DM556) sin micro-ROS.
//
// Conexion (anodo comun con transistores NPN o ULN2003):
//   5V del ESP32 -> PUL+, DIR+, ENA+
//   GPIO25 -> transistor -> PUL-
//   GPIO26 -> transistor -> DIR-
//   GPIO27 -> transistor -> ENA-
// Con el transistor, GPIO en HIGH = optoacoplador encendido.
// En ENA eso significa driver DESHABILITADO (motor suelto).
//
// DIP del DM556: 3200 pulsos/vuelta (si pones otro valor, cambia PASOS_POR_VUELTA).
//
// Monitor serie a 115200. Teclas:
//   d = +90 grados    a = -90 grados
//   w = +1 vuelta     s = -1 vuelta
//   e = habilitar / soltar el motor

#include <Arduino.h>

const int PIN_PUL = 25;
const int PIN_DIR = 26;
const int PIN_ENA = 27;

const long PASOS_POR_VUELTA = 3200;  // debe coincidir con SW5-SW8
const float VEL_MIN = 200.0f;        // pasos/s al arrancar y al frenar
const float VEL_MAX = 1600.0f;       // pasos/s = media vuelta por segundo (30 RPM)
const float ACEL = 3200.0f;          // pasos/s^2

bool habilitado = false;

void habilitar(bool on) {
  habilitado = on;
  digitalWrite(PIN_ENA, on ? LOW : HIGH);  // ENA activo = driver deshabilitado
  delay(10);                               // ENA debe ir antes que DIR/PUL
  Serial.println(on ? "Motor HABILITADO (eje trabado)" : "Motor SUELTO (eje libre)");
}

// Movimiento bloqueante con rampa de aceleracion y frenado
void mover(long pasos) {
  if (!habilitado) {
    Serial.println("El motor esta suelto: pulsa 'e' para habilitarlo");
    return;
  }
  Serial.printf("Moviendo %ld pasos (%.1f grados)...\n", pasos, pasos * 360.0f / PASOS_POR_VUELTA);

  digitalWrite(PIN_DIR, pasos >= 0 ? HIGH : LOW);
  delayMicroseconds(50);  // DIR estable al menos 5 us antes del pulso

  long n = labs(pasos);
  float v = VEL_MIN;
  for (long i = 0; i < n; i++) {
    long faltan = n - i;
    float pasos_para_frenar = (v * v - VEL_MIN * VEL_MIN) / (2.0f * ACEL);
    if (faltan <= pasos_para_frenar) v -= ACEL / v;
    else if (v < VEL_MAX) v += ACEL / v;
    v = constrain(v, VEL_MIN, VEL_MAX);

    uint32_t periodo_us = (uint32_t)(1e6f / v);
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(10);  // el DM556 pide al menos 2.5 us
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(periodo_us - 10);
  }
  Serial.println("Listo");
}

void imprimirAyuda() {
  Serial.println();
  Serial.println("=== Prueba motor a pasos (DM556) ===");
  Serial.println("d = +90 grados   a = -90 grados");
  Serial.println("w = +1 vuelta    s = -1 vuelta");
  Serial.println("e = habilitar / soltar el motor");
}

void setup() {
  pinMode(PIN_PUL, OUTPUT);
  digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);
  pinMode(PIN_ENA, OUTPUT);

  Serial.begin(115200);
  delay(500);
  imprimirAyuda();
  habilitar(true);
}

void loop() {
  if (!Serial.available()) return;
  char c = tolower(Serial.read());
  switch (c) {
    case 'd': mover(PASOS_POR_VUELTA / 4); break;
    case 'a': mover(-PASOS_POR_VUELTA / 4); break;
    case 'w': mover(PASOS_POR_VUELTA); break;
    case 's': mover(-PASOS_POR_VUELTA); break;
    case 'e': habilitar(!habilitado); break;
    case '\r':
    case '\n':
    case ' ': break;
    default: imprimirAyuda(); break;
  }
}
