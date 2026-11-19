// main.cpp -- Brazo robotico ESP32 + micro-ROS (port del main.py de MicroPython)
//
// Motores: la base es un NEMA 23 (motor a pasos con driver DM556) y encima van
// 5 servos (el 5 es la garra). La base gira mientras se mantiene a/d o el stick,
// sin pasos de grados fijos.
//
// Misma logica que la version de Thonny:
//   - 5 servos (el 5 es la garra; el servo 6 de Thonny no existe en el brazo real)
//   - MPU: mientras el sensor este inclinado mas alla de ZONA_MUERTA, el servo
//     avanza MPU_PASO grados por muestra en esa direccion, con movimiento suave.
//     Eje X mueve el servo 1, eje Y sube/baja el brazo (servo 2).
//     (En Thonny FIX 1/FIX 2 pedian moverse mas que la zona muerta de posicion
//     pero cada muestra solo sumaba 2 grados, asi que el MPU casi nunca movia.)
//   - FIX 3: MPU cada 80 ms
//   - FIX 4: zona muerta del acelerometro 6000
//   - FIX 5: media movil de 5 muestras
//   - FIX 6/7: zona muerta mayor y pulsos recortados en servo 1
//   - FIX 8: anguloSeguroS1() salta las zonas de temblor
//   - VL53L0X: objeto < 10 cm -> abre la pinza
//
// Diferencias obligadas por micro-ROS:
//   - El teclado serie se reemplaza por topics (el puerto serie lo usa micro-ROS).
//   - mover_servo_suave() ya no bloquea: el loop avanza un poco cada TICK_MOV_US.
//     Asi el executor nunca se queda sin tiempo y no se pierde el agente.
//   - Servos y sensores funcionan aunque el agente este desconectado.
//
// Movimiento suave (contra el juego de los engranes):
//   - Cada servo acelera y frena en rampa (AJUSTE_MOV) en vez de arrancar y
//     parar en seco: ese golpe es lo que hace rebotar el brazo en el juego.
//   - Curva S opcional (suave_ms): redondea el inicio y el fin de cada rampa, para
//     que el freno no entre de golpe y lance el brazo contra el otro lado del juego.
//   - Las teclas siguen sumando STEP al objetivo, pero el servo las sigue de corrido,
//     sin frenar entre una y otra, hasta su vel maxima. Al soltar frena en unos grados.
//   - Angulos con decimales: el pulso cambia poco a poco, sin escalones de 1 grado.
//
// Alimentacion:
//   - Al arrancar los servos se activan uno por uno (ARRANQUE_ESCALONADO_MS)
//     para no tumbar la fuente con el pico de todos a la vez.
//   - Un servo apagado no recibe pulsos: queda suelto y casi no consume.
//
// Regreso lento a "inicio" al arrancar:
//   - Los servos no dicen donde estan, asi que su ultima posicion se guarda en
//     flash (NVS) cuando llevan GUARDAR_QUIETO_MS quietos. Al arrancar, el primer
//     pulso va a esa posicion (sin salto) y de ahi van a "inicio" a VEL_REGRESO.
//   - Se guarda como pulso (us), no como angulo: sigue valiendo aunque se
//     recalibre un servo (cero, sentido, min_us, max_us).
//
// Topics (namespace /brazo):
//   Suscripciones:
//     /brazo/tecla      std_msgs/String           a/d base (mientras llegue la tecla), x soltar/trabar base, w/s servo 2,
//                                                 t/g servo 1, q/e servo 3, z/c servo 4, r/f garra
//     /brazo/base       std_msgs/Int32            velocidad de la base en % de VEL_BASE_MAX
//                                                 (-100..100, 0 = frenar). Hay que repetirla:
//                                                 si no llega en BASE_VENCE_MS, la base frena.
//     /brazo/servo      std_msgs/Int32MultiArray  [servo, angulo] | [servo, angulo, vel] | [a1..a5]
//     /brazo/activar    std_msgs/Int32MultiArray  [servo, 0|1] | [a1..a5] con 0 = apagado, 1 = activo
//   Publicaciones:
//     /brazo/estado     std_msgs/Int32MultiArray  [s1..s5, base] angulos actuales (la base en
//                                                 grados desde donde estaba al encender)
//     /brazo/distancia  std_msgs/Int32            VL53L0X en mm
//     /brazo/imu        std_msgs/Int32MultiArray  [acc_x, acc_y] filtrados (equivale a DEBUG_S1)
//     /brazo/diag       std_msgs/String           dispositivos I2C, sensores y servos activos

#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <VL53L0X.h>
#include <Preferences.h>
#include <FastAccelStepper.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "Este firmware espera board_microros_transport = serial en platformio.ini"
#endif

// --- CONFIGURACION MAESTRA ---
const int FREQ = 50;
const int PERIOD_US = 20000;
const int PWM_BITS = 16;
const int NUM_SERVOS = 5;
const int GARRA = 5;  // servo de la garra

// Escala fisica: 0..180 grados = min_us..max_us (fuera de ese rango se extrapola).
// El angulo que se ve en topics y teclas es: fisico = cero + sentido * angulo.
// Con sentido = -1 el servo gira al reves (montado en espejo).
// lim_min, lim_max e inicio van en el angulo que se ve.
struct ConfigServo {
  uint8_t pin;
  int min_us;
  int max_us;
  int lim_min;
  int lim_max;
  int inicio;   // angulo al arrancar
  int cero;     // angulo fisico donde se ve 0
  int sentido;  // +1 normal, -1 invertido
};

// Indice 0 = servo 1 ... indice 4 = servo 5 (garra)
const ConfigServo CONFIG_SERVOS[NUM_SERVOS] = {
  //pin min_us max_us lim_min lim_max inicio cero sentido
  // Inicios = postura de reposo elegida en el brazo real: [74, 100, 106, 100, ...].
  // servo 1: tope fisico en 42 -> minimo 44. Maximo 198 (2580 us): probado sin
  // topar. Arriba de 180 el pulso sigue la misma escala (10 us por grado).
  {15, 600,  2400, 44,   198, 74,  0,  +1},
  // servo 2 (codo): invertido tras el ajuste fisico. 0 = antebrazo recto (fisico 70).
  // "w" (levantar) -> negativos hasta -110 (fisico 180, 2500 us).
  // "s" (bajar)    -> positivos hasta 100 (516 us, cerca del minimo del servo).
  {2,  800,  2500, -110, 100, 100, 70, -1},
  // servo 3: escala corta (500-1000 us), cada grado de aqui es ~1/4 de grado real.
  {4,  500,  1000, 0,    180, 106, 0,  +1},
  // servo 4: antes 900-2000 us (solo ~110 grados de giro). Ahora el recorrido
  // completo del servo. Inicio 91 = 1511 us, la misma postura que el 100 de antes.
  {16, 500,  2500, 0,    180, 91,  0,  +1},
  // servo 5 = garra, solo poco a poco con r/f (cerrar de golpe contra algo
  // grande forzaba el servo). Escala fina: 5 us por grado.
  // Horn remontado (antes el servo llegaba a su extremo con la garra apenas abierta).
  // Calibrado en el brazo: 12 = abierta del todo (560 us), 152 = cerrada (1260 us).
  // "f" abre, "r" cierra. Arranca abierta.
  {17, 500,  1400, 12,   152, 12,  0,  +1},
};

// Como se mueve cada servo, para que el juego de los engranes no lo haga rebotar.
//   acel:     grados/s^2 al arrancar y frenar. Menos = mas suave, pero frena mas largo.
//   vel:      grados/s maximos con tecla sostenida o mando a fondo.
//   suave_ms: curva S, redondea el inicio y fin de cada rampa (0 = sin curva).
//             Mas = mas suave, pero el servo va ese tiempo detras de la rampa.
// Si un servo todavia rebota al arrancar o al soltar: bajale acel o subele suave_ms.
// Si tarda mucho en frenar: al reves.
struct AjusteMov {
  int acel;
  int vel;
  int suave_ms;
};
const AjusteMov AJUSTE_MOV[NUM_SERVOS] = {
  //acel vel suave_ms
  // servo 1 (MG995): sus engranes tienen juego y al frenar el brazo se sigue de largo.
  // Con 150 / 40 / 0 todavia rebotaba al arrancar y al soltar. En un modelo del
  // juego (2 grados de hueco) esto bajo el choque al arrancar de 27 a ~0 g/s y al
  // soltar de 15 a 4. Si se siente muy lenta, subele vel (25 o 30).
  {50,  20, 150},
  {150, 40, 0},   // servo 2 (codo)
  {300, 40, 0},   // servo 3
  {300, 40, 0},   // servo 4
  {300, 40, 0},   // servo 5 (garra)
};

const int STEP = 2;
const float VEL = 120.0f;        // grados/s maximos de un movimiento a un angulo (/brazo/servo)
const uint32_t TICK_MOV_US = 5000;  // cada cuanto se recalcula el movimiento

// Garra abierta (angulo del servo 5): a donde la lleva el sensor de distancia.
const int GRIP_OPEN_ANG = 12;

// FIX 8: zonas problematicas del servo 1
struct Zona { int min; int max; };
const Zona ZONAS_PROBLEMATICAS_S1[] = {
  {85, 95},    // zona tipica de punto muerto mecanico central
  {0,  20},    // extremo inferior (alta carga mecanica)
  // {160, 180} quitada tras el ajuste fisico: el servo 1 necesita bajar mas de 159
};

// VL53L0X
const uint32_t SENSOR_SAMPLE_MS = 200;
const int DIST_THRESH_MM = 100;

// MPU6050 (GY-521)
const uint8_t MPU_ADDR = 0x68;
const uint32_t MPU_SAMPLE_MS = 80;  // FIX 3
const int ZONA_MUERTA = 6000;       // FIX 4
const int ACC_BUFFER_SIZE = 5;      // FIX 5
const int MPU_PASO = 2;             // grados por muestra: 2 cada 80 ms = 25 grados/s
// Servo que mueve cada eje del MPU y su sentido (cambiar el signo para invertirlo).
// En Thonny el eje Y movia el servo 3; en el brazo real subir/bajar es el servo 2.
const int MPU_SERVO_X = 1, MPU_SIGNO_X = +1;  // inclinar a los lados: servo 1
const int MPU_SERVO_Y = 2, MPU_SIGNO_Y = +1;  // inclinar adelante/atras: sube/baja

// Base: NEMA 23 + DM556, mismas conexiones que pruebas/prueba_motor. Cada senal
// pasa por un transistor NPN: GPIO en HIGH = optoacoplador encendido.
const uint8_t PIN_BASE_PUL = 25;
const uint8_t PIN_BASE_DIR = 26;
const uint8_t PIN_BASE_ENA = 27;            // HIGH = driver deshabilitado (motor suelto)
const float PASOS_VUELTA_BASE = 3200.0f;    // DIP SW5-SW8 del DM556
const float RELACION_BASE = 1.0f;           // vueltas del motor por vuelta de la base (banda/engranes)
const bool SENTIDO_BASE = true;             // cambialo si "d" gira la base al reves
const float VEL_BASE_MAX = 40.0f;           // grados/s con el mando a fondo (100 %)
const float ACEL_BASE = 60.0f;              // grados/s^2: arranca y frena gradual
const float BASE_SUAVE_GRADOS = 3.0f;       // al arrancar, la aceleracion crece poco a poco en estos grados
// Teclado: a/d empiezan despacio (toques finos) y aceleran si se mantienen
const float BASE_TECLA_INI = 25.0f;         // % de VEL_BASE_MAX al apretar
const float BASE_TECLA_FIN = 75.0f;         // % tras mantenerla BASE_TECLA_RAMPA_MS
const uint32_t BASE_TECLA_RAMPA_MS = 1500;
const uint32_t BASE_VENCE_MS = 350;         // sin ordenes en este tiempo -> la base frena

// Alimentacion de servos
const uint32_t ARRANQUE_ESCALONADO_MS = 700;
const float VEL_REGRESO = 15.0f;            // grados/s del regreso a "inicio" al arrancar
const uint32_t GUARDAR_QUIETO_MS = 1000;    // quietos este tiempo -> se guarda la posicion

// Telemetria
const uint32_t ESTADO_PUB_MS = 100;
const uint32_t DIAG_PUB_MS = 2000;
const uint32_t REINTENTO_SENSORES_MS = 3000;
const uint8_t TOF_ADDR = 0x29;

// =========================================================
// SERVOS: PWM + movimiento suave no bloqueante
// =========================================================
struct Movimiento {
  float pos;     // angulo de la rampa (con decimales)
  float vel;     // grados/s de la rampa, con signo
  int objetivo;  // a donde va
  float vmax;    // grados/s maximos de este movimiento
  float acel;    // grados/s^2 de este movimiento
  float salida;  // angulo que recibe el servo: la rampa pasada por la curva S
};

Movimiento mov[NUM_SERVOS];
bool activo[NUM_SERVOS] = {false};

// Arranque escalonado: siguiente servo por activar (> NUM_SERVOS = terminado)
int siguiente_activar = 1;
uint32_t ultimo_activar_ms = 0;

const ConfigServo &cfg(int servo) { return CONFIG_SERVOS[servo - 1]; }

int limitar(int servo, int angulo) {
  const ConfigServo &c = cfg(servo);
  if (angulo < c.lim_min) angulo = c.lim_min;
  if (angulo > c.lim_max) angulo = c.lim_max;
  return angulo;
}

// Pulso de un angulo fisico (escala 0..180 = min_us..max_us)
int fisicoAMicros(int servo, float fisico) {
  const ConfigServo &c = cfg(servo);
  return (int)(c.min_us + (c.max_us - c.min_us) * (fisico / 180.0f));
}

int anguloAMicros(int servo, int angulo) {
  const ConfigServo &c = cfg(servo);
  angulo = limitar(servo, angulo);
  return fisicoAMicros(servo, c.cero + c.sentido * angulo);
}

// Igual, con decimales: el movimiento suave pasa por angulos intermedios
float anguloAMicrosF(int servo, float angulo) {
  const ConfigServo &c = cfg(servo);
  angulo = constrain(angulo, (float)c.lim_min, (float)c.lim_max);
  float fisico = c.cero + c.sentido * angulo;
  return c.min_us + (c.max_us - c.min_us) * (fisico / 180.0f);
}

// Inverso de anguloAMicros (sin limitar), para recuperar la posicion guardada
int microsAAngulo(int servo, int us) {
  const ConfigServo &c = cfg(servo);
  float fisico = (us - c.min_us) * 180.0f / (c.max_us - c.min_us);
  return (int)lroundf((fisico - c.cero) * c.sentido);
}

void escribirDuty(int servo, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(cfg(servo).pin, duty);
#else
  ledcWrite(servo - 1, duty);
#endif
}

void escribirServo(int servo, float angulo) {
  if (!activo[servo - 1]) return;  // apagado: sin pulsos
  float us = anguloAMicrosF(servo, angulo);
  escribirDuty(servo, (uint32_t)lroundf(us * ((1u << PWM_BITS) - 1) / PERIOD_US));
}

int anguloActual(int servo) { return (int)lroundf(mov[servo - 1].salida); }

bool enMovimiento(int servo) {
  const Movimiento &m = mov[servo - 1];
  return m.pos != m.objetivo || m.vel != 0.0f || m.salida != m.pos;
}

// Al activarse, el servo va a la ultima posicion guardada
void activarServo(int servo, bool on) {
  activo[servo - 1] = on;
  if (on) escribirServo(servo, mov[servo - 1].salida);
  else escribirDuty(servo, 0);
}

void iniciarServo(int servo) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(cfg(servo).pin, FREQ, PWM_BITS);
#else
  ledcSetup(servo - 1, FREQ, PWM_BITS);
  ledcAttachPin(cfg(servo).pin, servo - 1);
#endif
}

// Equivalente a mover_servo_suave(), pero solo fija el objetivo;
// actualizarMovimientos() lo mueve desde el loop. acel <= 0: la de la tabla.
void moverServoSuave(int servo, int angulo_obj, float velocidad_deg_por_s = 60.0f, float acel = 0.0f) {
  Movimiento &m = mov[servo - 1];
  m.objetivo = limitar(servo, angulo_obj);
  m.vmax = max(0.5f, velocidad_deg_por_s);
  m.acel = acel > 0.0f ? acel : (float)AJUSTE_MOV[servo - 1].acel;
}

// Equivalente a mover_todos_suave(): todos llegan al mismo tiempo.
// Velocidad y aceleracion a escala de lo que recorre cada uno, con la
// aceleracion del mas suave para que ninguno pase de la suya.
void moverTodosSuave(const int objetivos[NUM_SERVOS], float velocidad_deg_por_s = 60.0f) {
  float maxdiff = 0.0f, acel = 1e9f;
  for (int i = 0; i < NUM_SERVOS; i++) {
    float d = fabsf(limitar(i + 1, objetivos[i]) - mov[i].pos);
    if (d == 0.0f) continue;
    maxdiff = max(maxdiff, d);
    acel = min(acel, (float)AJUSTE_MOV[i].acel);
  }
  if (maxdiff == 0.0f) return;
  for (int i = 0; i < NUM_SERVOS; i++) {
    float f = fabsf(limitar(i + 1, objetivos[i]) - mov[i].pos) / maxdiff;
    if (f == 0.0f) continue;
    moverServoSuave(i + 1, objetivos[i], velocidad_deg_por_s * f, acel * f);
  }
}

// Perfil trapezoidal: acelera hasta vmax y frena a tiempo para llegar al
// objetivo sin velocidad. Si el objetivo cambia a medio camino (mas teclas,
// la tecla contraria), la velocidad cambia en rampa, nunca de golpe.
void actualizarMovimientos() {
  static uint32_t ultimo_us = 0;
  uint32_t ahora = micros();
  uint32_t transcurrido = ahora - ultimo_us;
  if (transcurrido < TICK_MOV_US) return;
  ultimo_us = ahora;
  // Si el loop se atoro (p. ej. un ping al agente), mejor ir lento que dar un salto
  float dt = min(transcurrido, (uint32_t)25000) * 1e-6f;
  for (int i = 0; i < NUM_SERVOS; i++) {
    Movimiento &m = mov[i];
    float err = m.objetivo - m.pos;
    if (err != 0.0f || m.vel != 0.0f) {
      float dv = m.acel * dt;  // cuanto puede cambiar la velocidad en este tick
      // La mas rapida con la que todavia alcanza a frenar justo en el objetivo,
      // contando que se frena a saltos de un tick (v^2/2a + v*dt/2 = err). Con la
      // formula continua llegaba un poco pasado y regresaba: un cambio de sentido,
      // justo lo que hace brincar el juego.
      float media = 0.5f * dv;
      float v_frenado = -media + sqrtf(media * media + 2.0f * m.acel * fabsf(err));
      v_frenado = copysignf(min(m.vmax, v_frenado), err);
      m.vel += constrain(v_frenado - m.vel, -dv, dv);
      float avance = m.vel * dt;
      if (avance * err >= 0.0f && fabsf(avance) >= fabsf(err) && fabsf(m.vel) <= 3.0f * dv) {
        m.pos = m.objetivo;  // llego, ya casi sin velocidad
        m.vel = 0.0f;
      } else {
        m.pos += avance;  // si el objetivo se echo para atras, se pasa un poco y regresa
      }
      const ConfigServo &c = cfg(i + 1);
      if (m.pos < c.lim_min || m.pos > c.lim_max) {
        m.pos = constrain(m.pos, (float)c.lim_min, (float)c.lim_max);
        m.vel = 0.0f;
      }
    } else if (m.salida == m.pos) {
      continue;  // quieto
    }
    // Curva S: la salida sigue a la rampa con un filtro de primer orden. La
    // aceleracion ya no cambia de golpe (al empezar a frenar entra poco a poco)
    // y, como el filtro nunca se pasa, tampoco agrega cambios de sentido.
    int suave_ms = AJUSTE_MOV[i].suave_ms;
    if (suave_ms > 0 && fabsf(m.pos - m.salida) > 0.01f)
      m.salida += (m.pos - m.salida) * (1.0f - expf(-dt * 1000.0f / suave_ms));
    else
      m.salida = m.pos;
    escribirServo(i + 1, m.salida);
  }
}

// Arranque: cada servo recibe su primer pulso donde quedo (ultima posicion
// guardada, sin salto) y de ahi va despacio a su "inicio".
void arranqueEscalonado(uint32_t ahora) {
  if (siguiente_activar > NUM_SERVOS || ahora - ultimo_activar_ms < ARRANQUE_ESCALONADO_MS) return;
  ultimo_activar_ms = ahora;
  int servo = siguiente_activar++;
  activarServo(servo, true);
  moverServoSuave(servo, cfg(servo).inicio, VEL_REGRESO);
}

// Ultimas posiciones en flash (NVS), como pulso en us. Se escriben solo cuando
// todos los servos llevan GUARDAR_QUIETO_MS quietos y algo cambio, para no gastar la flash.
Preferences prefs;
int16_t pos_guardadas[NUM_SERVOS];  // angulos de la ultima vez que se guardo
uint32_t ultimo_movimiento_ms = 0;

bool cargarPosiciones(int pos[NUM_SERVOS]) {
  // Caben mas de NUM_SERVOS: el firmware anterior guardaba 6 (el 6 ya no existe).
  int16_t buf[8];
  size_t n = prefs.getBytes("us", buf, sizeof(buf)) / sizeof(buf[0]);
  if (n < (size_t)NUM_SERVOS) return false;  // primera vez
  for (int i = 0; i < NUM_SERVOS; i++) pos[i] = microsAAngulo(i + 1, buf[i]);
  return true;
}

void guardarPosiciones(uint32_t ahora) {
  bool cambio = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (enMovimiento(i + 1)) {
      ultimo_movimiento_ms = ahora;  // todavia se mueve
      return;
    }
    if (mov[i].objetivo != pos_guardadas[i]) cambio = true;
  }
  if (!cambio || ahora - ultimo_movimiento_ms < GUARDAR_QUIETO_MS) return;
  int16_t us[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    pos_guardadas[i] = mov[i].objetivo;  // quieto: pos == objetivo
    us[i] = anguloAMicros(i + 1, mov[i].objetivo);
  }
  prefs.putBytes("us", us, sizeof(us));
}

// FIX 8: si el angulo cae en una zona problematica lo saca al borde.
// direccion > 0 / < 0: hacia donde se mueve el servo (salta la zona central
// en lugar de rebotar). direccion = 0: borde mas cercano, como el original.
// Si el borde preferido queda fuera de lim_min/lim_max se usa el otro (pared).
int anguloSeguroS1(int angulo, int direccion) {
  const ConfigServo &c = cfg(1);
  for (const Zona &z : ZONAS_PROBLEMATICAS_S1) {
    if (angulo < z.min || angulo > z.max) continue;
    int abajo = z.min - 1;
    int arriba = z.max + 1;
    bool abajo_ok = abajo >= c.lim_min;
    bool arriba_ok = arriba <= c.lim_max;
    bool preferir_arriba = direccion != 0 ? direccion > 0 : angulo > (z.min + z.max) / 2;
    if (preferir_arriba) return arriba_ok ? arriba : (abajo_ok ? abajo : angulo);
    return abajo_ok ? abajo : (arriba_ok ? arriba : angulo);
  }
  return angulo;
}

// =========================================================
// BASE: NEMA 23 + DM556
// =========================================================
// Se controla por velocidad, no por grados: mientras lleguen ordenes gira, y si
// dejan de llegar (se solto la tecla o el stick, o se cayo el agente) frena en
// rampa. Los pulsos los genera el hardware del ESP32 (MCPWM) con FastAccelStepper:
// el loop puede atorarse unos ms con micro-ROS sin que el motor se trabe.
FastAccelStepperEngine motor_engine;
FastAccelStepper *base = nullptr;
float base_pct = 0.0f;       // velocidad pedida ahora (% de VEL_BASE_MAX, con signo)
uint32_t base_vence_ms = 0;  // si no llega otra orden antes, frena
bool base_trabada = false;   // driver habilitado: con fuerza (el "toc" al encender)

int32_t gradosAPasosBase(float grados) {
  return (int32_t)lroundf(grados * PASOS_VUELTA_BASE * RELACION_BASE / 360.0f);
}

int anguloBase() {
  if (!base) return 0;
  return (int)lroundf(base->getCurrentPosition() * 360.0f / (PASOS_VUELTA_BASE * RELACION_BASE));
}

void iniciarBase() {
  motor_engine.init();
  base = motor_engine.stepperConnectToPin(PIN_BASE_PUL);
  if (!base) return;
  base->setDirectionPin(PIN_BASE_DIR, SENTIDO_BASE, 50);  // DIR estable 50 us antes del pulso
  base->setEnablePin(PIN_BASE_ENA, true);                 // GPIO en LOW = driver habilitado
  base->setAutoEnable(false);
  base->enableOutputs();  // con torque desde el inicio: la base no se va de lado
  base_trabada = true;
  base->setAcceleration(gradosAPasosBase(ACEL_BASE));
  base->setLinearAcceleration(gradosAPasosBase(BASE_SUAVE_GRADOS));
}

// Tecla x (o boton Y del mando), como la "e" de pruebas/prueba_motor:
// suelta = driver deshabilitado, eje libre para moverlo a mano y sin calentar;
// trabada = con fuerza. Suelta, la base ignora las ordenes de giro.
void alternarTrabaBase() {
  static uint32_t ultimo_ms = 0;
  uint32_t ahora = millis();
  if (!base || ahora - ultimo_ms < 500) return;  // la tecla sostenida se repite: solo cuenta la primera
  ultimo_ms = ahora;
  base_pct = 0.0f;
  if (base_trabada) {
    base->forceStop();
    base->disableOutputs();
  } else {
    // Si la giraron a mano, el conteo ya no vale: el angulo vuelve a contar desde aqui
    base->forceStopAndNewPosition(0);
    base->enableOutputs();
  }
  base_trabada = !base_trabada;
}

// pct: -100..100 (% de VEL_BASE_MAX). vence_ms: cuanto vale la orden.
void comandarBase(float pct, uint32_t vence_ms) {
  if (!base || !base_trabada) return;
  pct = constrain(pct, -100.0f, 100.0f);
  base_vence_ms = millis() + vence_ms;
  bool igual = (pct == 0.0f) == (base_pct == 0.0f) && pct * base_pct >= 0.0f && fabsf(pct - base_pct) < 1.0f;
  if (igual) return;  // solo se renueva el plazo
  base_pct = pct;
  if (pct == 0.0f) {
    base->stopMove();  // frena con la rampa
    return;
  }
  base->setSpeedInHz(max((int32_t)1, gradosAPasosBase(fabsf(pct) / 100.0f * VEL_BASE_MAX)));
  // Si iba al otro lado, la libreria frena y luego invierte, sin golpe
  if (pct > 0.0f) base->runForward();
  else base->runBackward();
}

void vigilarBase(uint32_t ahora) {
  if (base_pct != 0.0f && (int32_t)(ahora - base_vence_ms) >= 0) comandarBase(0.0f, 0);
}

// a/d por /brazo/tecla. La terminal no avisa cuando se suelta una tecla: solo
// deja de repetirla. La primera pulsacion dura hasta que empieza la repeticion
// automatica (250 ms en este teclado); ya repitiendo, si no llega la siguiente
// en 3 intervalos se tomo como soltada.
uint32_t base_tecla_ms = 0, base_tecla_inicio_ms = 0;
int base_tecla_dir = 0;

void teclaBase(int dir) {
  uint32_t ahora = millis();
  uint32_t entre = ahora - base_tecla_ms;
  bool sigue = dir == base_tecla_dir && base_pct != 0.0f && entre < BASE_VENCE_MS;
  if (!sigue) base_tecla_inicio_ms = ahora;
  base_tecla_dir = dir;
  base_tecla_ms = ahora;
  float f = min(1.0f, (ahora - base_tecla_inicio_ms) / (float)BASE_TECLA_RAMPA_MS);
  float pct = BASE_TECLA_INI + (BASE_TECLA_FIN - BASE_TECLA_INI) * f;
  uint32_t vence = (sigue && entre < 100) ? max((uint32_t)120, 3 * entre) : BASE_VENCE_MS;
  comandarBase(dir * pct, vence);
}

// =========================================================
// CONTROL POR TECLAS (ahora llegan por /brazo/tecla)
// =========================================================
// Teclas y MPU suman delta al objetivo (pulsaciones seguidas se acumulan, como
// en MicroPython), pero sin que se adelante mas de lo que el servo alcanza a
// frenar a su vel maxima: la tecla sostenida da una velocidad pareja y al
// soltarla el brazo se detiene en unos grados.
// La velocidad sigue al ritmo de las teclas: con el mando apenas inclinado el
// servo va despacio y de corrido, en vez de dar un tiron por tecla y pararse a
// esperar la siguiente. Si se va quedando atras, acelera un poco para alcanzar.
const uint32_t TECLA_SUELTA_MS = 400;  // mas que esto entre teclas = toque suelto
uint32_t ultima_tecla_ms[NUM_SERVOS];
float vel_tecla[NUM_SERVOS];

void avanzarObjetivo(int servo, int delta) {
  const Movimiento &m = mov[servo - 1];
  const AjusteMov &aj = AJUSTE_MOV[servo - 1];
  float vel_max = aj.vel;
  uint32_t ahora = millis();
  uint32_t entre = ahora - ultima_tecla_ms[servo - 1];
  if (entre > TECLA_SUELTA_MS) vel_tecla[servo - 1] = vel_max;  // toque: lo limita acel
  else if (entre >= 2)  // < 2 ms: varias teclas en el mismo mensaje
    vel_tecla[servo - 1] = constrain(abs(delta) * 1000.0f / entre, 3.0f, vel_max);
  ultima_tecla_ms[servo - 1] = ahora;

  float adelanto = vel_max * vel_max / (2.0f * aj.acel) + STEP;
  int nuevo = m.objetivo + delta;
  if (delta > 0) nuevo = min(nuevo, max(m.objetivo, (int)floorf(m.pos + adelanto)));
  else nuevo = max(nuevo, min(m.objetivo, (int)ceilf(m.pos - adelanto)));
  if (servo == 1) nuevo = anguloSeguroS1(nuevo, delta);  // FIX 8
  float atraso = max(0.0f, fabsf(nuevo - m.pos) - 2 * STEP);
  moverServoSuave(servo, nuevo, min(vel_tecla[servo - 1] + atraso, vel_max));
}

void procesarChar(char ch) {
  int servo = 0, delta = 0;
  switch (ch) {
    // Antes (Thonny) w/s movian el servo 1 y a/d el servo 2, y el teleop las
    // intercambiaba. Ahora el firmware usa directo las teclas del teleop.
    case 'a': teclaBase(-1); return;  // base: gira mientras se mantenga
    case 'd': teclaBase(+1); return;
    case 'x': alternarTrabaBase(); return;  // soltar / trabar la base
    case 'w': servo = 2; delta = -STEP; break;  // levanta
    case 's': servo = 2; delta = +STEP; break;  // baja
    case 't': servo = 1; delta = +STEP; break;
    case 'g': servo = 1; delta = -STEP; break;
    case 'q': servo = 3; delta = +STEP; break;
    case 'e': servo = 3; delta = -STEP; break;
    case 'z': servo = 4; delta = -STEP; break;
    case 'c': servo = 4; delta = +STEP; break;
    case 'r': servo = GARRA; delta = +STEP; break;  // cierra
    case 'f': servo = GARRA; delta = -STEP; break;  // abre
    default:
      return;
  }
  avanzarObjetivo(servo, delta);
}

// =========================================================
// SENSORES
// =========================================================
VL53L0X tof;
bool tof_ok = false;
uint32_t last_sensor_ms = 0;
int32_t ultima_distancia = -1;
bool distancia_nueva = false;

bool mpu_ok = false;
uint32_t last_mpu_ms = 0;
int32_t acc_x = 0, acc_y = 0;
bool imu_nueva = false;

bool mpuIniciar() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1: despertar
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

// Lecturas crudas (+-2g), mismas unidades que AcX/AcY de mpu6050.py
bool mpuLeerAcc(int16_t &ax, int16_t &ay) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)4) != 4) return false;
  uint8_t xh = Wire.read(), xl = Wire.read();
  uint8_t yh = Wire.read(), yl = Wire.read();
  ax = (int16_t)((xh << 8) | xl);
  ay = (int16_t)((yh << 8) | yl);
  return true;
}

// FIX 5: media movil
void filtrarAcc(int32_t nuevo_x, int32_t nuevo_y, int32_t &fx, int32_t &fy) {
  static int32_t buf_x[ACC_BUFFER_SIZE] = {0};
  static int32_t buf_y[ACC_BUFFER_SIZE] = {0};
  static int idx = 0;
  buf_x[idx] = nuevo_x;
  buf_y[idx] = nuevo_y;
  idx = (idx + 1) % ACC_BUFFER_SIZE;
  int32_t sx = 0, sy = 0;
  for (int i = 0; i < ACC_BUFFER_SIZE; i++) {
    sx += buf_x[i];
    sy += buf_y[i];
  }
  fx = sx / ACC_BUFFER_SIZE;
  fy = sy / ACC_BUFFER_SIZE;
}

void tofIniciar() {
  tof.setTimeout(100);
  if (!tof.init()) return;
  tof.setMeasurementTimingBudget(40000);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);
  tof.startContinuous(SENSOR_SAMPLE_MS);
  tof_ok = true;
}

void muestrearTof(uint32_t ahora) {
  if (!tof_ok || ahora - last_sensor_ms < SENSOR_SAMPLE_MS) return;
  // Solo lee cuando hay medicion lista, para no bloquear el loop
  if ((tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  last_sensor_ms = ahora;
  uint16_t dist = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return;

  ultima_distancia = dist;
  distancia_nueva = true;
  if (dist < DIST_THRESH_MM && mov[GARRA - 1].objetivo != GRIP_OPEN_ANG)
    moverServoSuave(GARRA, GRIP_OPEN_ANG, VEL);
}

// Mientras el eje este inclinado mas alla de ZONA_MUERTA, el servo avanza.
// Se suma sobre el objetivo (no sobre la posicion actual) para que el avance
// se acumule muestra a muestra mientras dure la inclinacion.
void moverPorInclinacion(int32_t acc, int servo, int signo) {
  if (acc <= ZONA_MUERTA && acc >= -ZONA_MUERTA) return;
  int dir = (acc > 0 ? +1 : -1) * signo;
  avanzarObjetivo(servo, dir * MPU_PASO);
}

// ANTI-TEMBLOR COMPLETO
void muestrearMpu(uint32_t ahora) {
  if (!mpu_ok || ahora - last_mpu_ms < MPU_SAMPLE_MS) return;
  last_mpu_ms = ahora;
  int16_t rx, ry;
  if (!mpuLeerAcc(rx, ry)) return;
  filtrarAcc(rx, ry, acc_x, acc_y);
  imu_nueva = true;

  moverPorInclinacion(acc_x, MPU_SERVO_X, MPU_SIGNO_X);
  moverPorInclinacion(acc_y, MPU_SERVO_Y, MPU_SIGNO_Y);
}

// Diagnostico I2C: direcciones que responden, p. ej. "0x29 0x68"
char i2c_encontrados[48] = "sin escanear";
bool hay_tof = false, hay_mpu = false;

void escanearI2C() {
  size_t n = 0;
  hay_tof = hay_mpu = false;
  i2c_encontrados[0] = '\0';
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t r = Wire.endTransmission();
    if (r == 0) {
      if (addr == TOF_ADDR) hay_tof = true;
      if (addr == MPU_ADDR) hay_mpu = true;
      if (n < sizeof(i2c_encontrados))
        n += snprintf(i2c_encontrados + n, sizeof(i2c_encontrados) - n, "%s0x%02X", n ? " " : "", addr);
    } else if (r != 2) {
      // Timeout u otro error: bus trabado (SDA/SCL en corto, sin GND comun o sin pull-ups)
      snprintf(i2c_encontrados, sizeof(i2c_encontrados), "error de bus (codigo %u)", r);
      hay_tof = hay_mpu = false;
      return;
    }
  }
  if (n == 0) snprintf(i2c_encontrados, sizeof(i2c_encontrados), "ninguno");
}

// Si un sensor no arranco (o se conecto despues), se reintenta cada pocos segundos
void reintentarSensores(uint32_t ahora) {
  static uint32_t ultimo = 0;
  if ((tof_ok && mpu_ok) || ahora - ultimo < REINTENTO_SENSORES_MS) return;
  ultimo = ahora;
  escanearI2C();
  if (!tof_ok && hay_tof) tofIniciar();
  if (!mpu_ok && hay_mpu) mpu_ok = mpuIniciar();
}

// =========================================================
// micro-ROS
// =========================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
bool support_iniciado = false;

rcl_subscription_t sub_tecla, sub_base, sub_servo, sub_activar;
rcl_publisher_t pub_estado, pub_distancia, pub_imu, pub_diag;

std_msgs__msg__String msg_tecla;
char buf_tecla[32];
std_msgs__msg__Int32 msg_base;
std_msgs__msg__Int32MultiArray msg_servo;
int32_t buf_servo[8];
std_msgs__msg__Int32MultiArray msg_activar;
int32_t buf_activar[8];

std_msgs__msg__Int32MultiArray msg_estado;
int32_t buf_estado[NUM_SERVOS + 1];  // servos + base
std_msgs__msg__Int32 msg_distancia;
std_msgs__msg__Int32MultiArray msg_imu;
int32_t buf_imu[2];
std_msgs__msg__String msg_diag;
char buf_diag[128];

enum EstadoAgente { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, AGENTE_CONECTADO, AGENTE_DESCONECTADO };
EstadoAgente estado_agente = ESPERANDO_AGENTE;

void cbTecla(const void *msgin) {
  const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
  for (size_t i = 0; i < m->data.size; i++) procesarChar((char)tolower(m->data.data[i]));
}

// Mando: velocidad de la base en %, repetida mientras el stick este inclinado
void cbBase(const void *msgin) {
  const std_msgs__msg__Int32 *m = (const std_msgs__msg__Int32 *)msgin;
  comandarBase((float)m->data, BASE_VENCE_MS);
}

void cbServo(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msgin;
  const int32_t *d = m->data.data;
  if (m->data.size == 2 || m->data.size == 3) {
    int servo = d[0];
    if (servo < 1 || servo > NUM_SERVOS) return;
    int angulo = d[1];
    float vel = (m->data.size == 3 && d[2] > 0) ? (float)d[2] : VEL;
    if (servo == 1) angulo = anguloSeguroS1(angulo, angulo - mov[0].objetivo);
    moverServoSuave(servo, angulo, vel);
  } else if (m->data.size == NUM_SERVOS) {
    int objetivos[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; i++) objetivos[i] = d[i];
    objetivos[0] = anguloSeguroS1(objetivos[0], objetivos[0] - mov[0].objetivo);
    moverTodosSuave(objetivos, VEL);
  }
}

void cbActivar(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msgin;
  const int32_t *d = m->data.data;
  siguiente_activar = NUM_SERVOS + 1;  // un comando manual cancela el arranque escalonado
  if (m->data.size == 2) {
    if (d[0] < 1 || d[0] > NUM_SERVOS) return;
    activarServo(d[0], d[1] != 0);
  } else if (m->data.size == NUM_SERVOS) {
    for (int i = 0; i < NUM_SERVOS; i++) activarServo(i + 1, d[i] != 0);
  }
}

void prepararMensajes() {
  msg_tecla.data.data = buf_tecla;
  msg_tecla.data.capacity = sizeof(buf_tecla);
  msg_tecla.data.size = 0;

  msg_servo.data.data = buf_servo;
  msg_servo.data.capacity = sizeof(buf_servo) / sizeof(buf_servo[0]);
  msg_servo.data.size = 0;

  msg_activar.data.data = buf_activar;
  msg_activar.data.capacity = sizeof(buf_activar) / sizeof(buf_activar[0]);
  msg_activar.data.size = 0;

  msg_diag.data.data = buf_diag;
  msg_diag.data.capacity = sizeof(buf_diag);
  msg_diag.data.size = 0;

  msg_estado.data.data = buf_estado;
  msg_estado.data.capacity = NUM_SERVOS + 1;
  msg_estado.data.size = NUM_SERVOS + 1;

  msg_imu.data.data = buf_imu;
  msg_imu.data.capacity = 2;
  msg_imu.data.size = 2;
}

#define RCCHECK(fn) do { if ((fn) != RCL_RET_OK) return false; } while (0)

bool crearEntidades() {
  allocator = rcl_get_default_allocator();
  node = rcl_get_zero_initialized_node();
  executor = rclc_executor_get_zero_initialized_executor();
  sub_tecla = rcl_get_zero_initialized_subscription();
  sub_base = rcl_get_zero_initialized_subscription();
  sub_servo = rcl_get_zero_initialized_subscription();
  sub_activar = rcl_get_zero_initialized_subscription();
  pub_estado = rcl_get_zero_initialized_publisher();
  pub_distancia = rcl_get_zero_initialized_publisher();
  pub_imu = rcl_get_zero_initialized_publisher();
  pub_diag = rcl_get_zero_initialized_publisher();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  support_iniciado = true;
  RCCHECK(rclc_node_init_default(&node, "brazo_esp32", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_tecla, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "brazo/tecla"));
  RCCHECK(rclc_subscription_init_default(&sub_base, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "brazo/base"));
  RCCHECK(rclc_subscription_init_default(&sub_servo, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "brazo/servo"));
  RCCHECK(rclc_subscription_init_default(&sub_activar, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "brazo/activar"));

  // Best effort: un publish reliable puede bloquear esperando confirmacion
  RCCHECK(rclc_publisher_init_best_effort(&pub_estado, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "brazo/estado"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_distancia, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "brazo/distancia"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_imu, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "brazo/imu"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_diag, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "brazo/diag"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_tecla, &msg_tecla, &cbTecla, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_base, &msg_base, &cbBase, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo, &msg_servo, &cbServo, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_activar, &msg_activar, &cbActivar, ON_NEW_DATA));
  return true;
}

void destruirEntidades() {
  if (support_iniciado) {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    if (rmw_context) (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  }
  (void)rcl_publisher_fini(&pub_estado, &node);
  (void)rcl_publisher_fini(&pub_distancia, &node);
  (void)rcl_publisher_fini(&pub_imu, &node);
  (void)rcl_publisher_fini(&pub_diag, &node);
  (void)rcl_subscription_fini(&sub_tecla, &node);
  (void)rcl_subscription_fini(&sub_base, &node);
  (void)rcl_subscription_fini(&sub_servo, &node);
  (void)rcl_subscription_fini(&sub_activar, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  if (support_iniciado) (void)rclc_support_fini(&support);
  support_iniciado = false;
}

void publicarTelemetria(uint32_t ahora) {
  static uint32_t ultimo_estado = 0;
  if (ahora - ultimo_estado >= ESTADO_PUB_MS) {
    ultimo_estado = ahora;
    for (int i = 0; i < NUM_SERVOS; i++) buf_estado[i] = anguloActual(i + 1);
    buf_estado[NUM_SERVOS] = anguloBase();
    (void)rcl_publish(&pub_estado, &msg_estado, NULL);
  }
  if (distancia_nueva) {
    distancia_nueva = false;
    msg_distancia.data = ultima_distancia;
    (void)rcl_publish(&pub_distancia, &msg_distancia, NULL);
  }
  if (imu_nueva) {
    imu_nueva = false;
    buf_imu[0] = acc_x;
    buf_imu[1] = acc_y;
    (void)rcl_publish(&pub_imu, &msg_imu, NULL);
  }
  static uint32_t ultimo_diag = 0;
  if (ahora - ultimo_diag >= DIAG_PUB_MS) {
    ultimo_diag = ahora;
    char activos[NUM_SERVOS + 1];
    for (int i = 0; i < NUM_SERVOS; i++) activos[i] = activo[i] ? '1' : '0';
    activos[NUM_SERVOS] = '\0';
    int n = snprintf(buf_diag, sizeof(buf_diag), "i2c: %s | tof: %s | mpu: %s | servos activos: %s | base: %s",
                     i2c_encontrados, tof_ok ? "ok" : "no", mpu_ok ? "ok" : "no", activos, !base ? "no" : (base_trabada ? "trabada" : "suelta"));
    msg_diag.data.size = min((size_t)n, sizeof(buf_diag) - 1);
    (void)rcl_publish(&pub_diag, &msg_diag, NULL);
  }
}

// Reconexion automatica: si el agente se cae, el brazo sigue funcionando
// y las entidades se recrean cuando vuelve.
void gestionarMicroRos(uint32_t ahora) {
  static uint32_t ultimo_ping = 0;
  switch (estado_agente) {
    case ESPERANDO_AGENTE:
      if (ahora - ultimo_ping >= 500) {
        ultimo_ping = ahora;
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) estado_agente = AGENTE_DISPONIBLE;
      }
      break;
    case AGENTE_DISPONIBLE:
      if (crearEntidades()) {
        estado_agente = AGENTE_CONECTADO;
      } else {
        destruirEntidades();
        estado_agente = ESPERANDO_AGENTE;
      }
      break;
    case AGENTE_CONECTADO:
      if (ahora - ultimo_ping >= 1000) {
        ultimo_ping = ahora;
        if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
          estado_agente = AGENTE_DESCONECTADO;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      publicarTelemetria(ahora);
      break;
    case AGENTE_DESCONECTADO:
      destruirEntidades();
      estado_agente = ESPERANDO_AGENTE;
      break;
  }
}

// ----- SETUP / LOOP -----
void setup() {
  // Cada servo parte de su ultima posicion guardada (o de "inicio" la primera vez).
  // Los pulsos empiezan servo por servo desde arranqueEscalonado(), que los lleva
  // despacio a "inicio" (equivale a init_pos()).
  prefs.begin("brazo", false);
  int guardada[NUM_SERVOS];
  bool hay_guardada = cargarPosiciones(guardada);
  for (int s = 1; s <= NUM_SERVOS; s++) {
    iniciarServo(s);
    int desde = limitar(s, hay_guardada ? guardada[s - 1] : cfg(s).inicio);
    mov[s - 1] = {(float)desde, 0.0f, desde, VEL, (float)AJUSTE_MOV[s - 1].acel, (float)desde};
    pos_guardadas[s - 1] = desde;
  }
  iniciarBase();

  Wire.begin(21, 22, 100000);
  Wire.setTimeOut(20);  // un bus trabado no debe congelar el loop
  escanearI2C();
  if (hay_tof) tofIniciar();
  if (hay_mpu) mpu_ok = mpuIniciar();

  // El puerto serie queda reservado para micro-ROS: no usar Serial.print()
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  prepararMensajes();
}

void loop() {
  uint32_t ahora = millis();
  gestionarMicroRos(ahora);
  arranqueEscalonado(ahora);
  actualizarMovimientos();
  vigilarBase(ahora);
  guardarPosiciones(ahora);
  reintentarSensores(ahora);
  muestrearTof(ahora);
  muestrearMpu(ahora);
}
