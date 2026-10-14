// main.cpp -- Brazo robotico ESP32 + micro-ROS (port del main.py de MicroPython)
//
// Misma logica que la version de Thonny:
//   - 6 servos con CONFIG_SERVOS identico (min_us/max_us/lim_min/lim_max)
//   - MPU: mientras el sensor este inclinado mas alla de ZONA_MUERTA, el servo
//     avanza MPU_PASO grados por muestra en esa direccion, con movimiento suave.
//     Eje X gira la base (servo 1), eje Y sube/baja el brazo (servo 2).
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
//   - mover_servo_suave() ya no bloquea: cada vuelta del loop avanza un paso.
//     Asi el executor nunca se queda sin tiempo y no se pierde el agente.
//   - Servos y sensores funcionan aunque el agente este desconectado.
//
// Alimentacion:
//   - Al arrancar los servos se activan uno por uno (ARRANQUE_ESCALONADO_MS)
//     para no tumbar la fuente con el pico de los 6 a la vez.
//   - Un servo apagado no recibe pulsos: queda suelto y casi no consume.
//
// Topics (namespace /brazo):
//   Suscripciones:
//     /brazo/tecla      std_msgs/String           mismas teclas que en Thonny (w s a d q e z c r f, espacio)
//     /brazo/servo      std_msgs/Int32MultiArray  [servo, angulo] | [servo, angulo, vel] | [a1..a6]
//     /brazo/garra      std_msgs/Bool             true = cerrar, false = abrir
//     /brazo/activar    std_msgs/Int32MultiArray  [servo, 0|1] | [a1..a6] con 0 = apagado, 1 = activo
//   Publicaciones:
//     /brazo/estado     std_msgs/Int32MultiArray  angulos actuales [s1..s6]
//     /brazo/distancia  std_msgs/Int32            VL53L0X en mm
//     /brazo/imu        std_msgs/Int32MultiArray  [acc_x, acc_y] filtrados (equivale a DEBUG_S1)
//     /brazo/diag       std_msgs/String           dispositivos I2C, sensores y servos activos

#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <VL53L0X.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
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
const int NUM_SERVOS = 6;

struct ConfigServo {
  uint8_t pin;
  int min_us;
  int max_us;
  int lim_min;
  int lim_max;
};

// Indice 0 = servo 1 ... indice 5 = servo 6
const ConfigServo CONFIG_SERVOS[NUM_SERVOS] = {
  {15, 600,  2400, 15, 165},  // servo 1: FIX 6/7
  {2,  800,  2500, 70, 180},
  {4,  500,  1000, 0,  180},
  {16, 900,  2000, 0,  180},
  {17, 500,  2500, 0,  180},
  {5,  1100, 2700, 0,  180},
};

const int STEP = 2;
const float VEL = 120.0f;
const int PASO = 1;

const int GRIP_OPEN_ANG = 0;
const int GRIP_CLOSE_ANG = 90;
bool grip_closed = false;

// FIX 8: zonas problematicas del servo 1
struct Zona { int min; int max; };
const Zona ZONAS_PROBLEMATICAS_S1[] = {
  {85, 95},    // zona tipica de punto muerto mecanico central
  {0,  20},    // extremo inferior (alta carga mecanica)
  {160, 180},  // extremo superior (alta carga mecanica)
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
const int MPU_SERVO_X = 1, MPU_SIGNO_X = +1;  // inclinar a los lados: gira la base
const int MPU_SERVO_Y = 2, MPU_SIGNO_Y = +1;  // inclinar adelante/atras: sube/baja

// Alimentacion de servos
const uint32_t ARRANQUE_ESCALONADO_MS = 700;

// Telemetria
const uint32_t ESTADO_PUB_MS = 100;
const uint32_t DIAG_PUB_MS = 2000;
const uint32_t REINTENTO_SENSORES_MS = 3000;
const uint8_t TOF_ADDR = 0x29;

// =========================================================
// SERVOS: PWM + movimiento suave no bloqueante
// =========================================================
struct Movimiento {
  int actual;
  int objetivo;
  int paso;
  uint32_t intervalo_us;
  uint32_t ultimo_us;
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

int anguloAMicros(int servo, int angulo) {
  const ConfigServo &c = cfg(servo);
  angulo = limitar(servo, angulo);
  return (int)(c.min_us + (c.max_us - c.min_us) * (angulo / 180.0f));
}

void escribirDuty(int servo, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(cfg(servo).pin, duty);
#else
  ledcWrite(servo - 1, duty);
#endif
}

void escribirServo(int servo, int angulo) {
  if (!activo[servo - 1]) return;  // apagado: sin pulsos
  uint32_t us = anguloAMicros(servo, angulo);
  escribirDuty(servo, (uint32_t)((uint64_t)us * ((1u << PWM_BITS) - 1) / PERIOD_US));
}

// Al activarse, el servo va a la ultima posicion guardada
void activarServo(int servo, bool on) {
  activo[servo - 1] = on;
  if (on) escribirServo(servo, mov[servo - 1].actual);
  else escribirDuty(servo, 0);
}

void arranqueEscalonado(uint32_t ahora) {
  if (siguiente_activar > NUM_SERVOS || ahora - ultimo_activar_ms < ARRANQUE_ESCALONADO_MS) return;
  ultimo_activar_ms = ahora;
  activarServo(siguiente_activar, true);
  siguiente_activar++;
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
// actualizarMovimientos() da los pasos desde el loop.
void moverServoSuave(int servo, int angulo_obj, float velocidad_deg_por_s = 60.0f, int paso_deg = 1) {
  Movimiento &m = mov[servo - 1];
  m.objetivo = limitar(servo, angulo_obj);
  m.paso = max(1, paso_deg);
  m.intervalo_us = (uint32_t)(1e6f * m.paso / max(0.0001f, velocidad_deg_por_s));
  m.ultimo_us = micros() - m.intervalo_us;  // primer paso inmediato, como el original
}

// Equivalente a mover_todos_suave(): todos llegan al mismo tiempo.
void moverTodosSuave(const int objetivos[NUM_SERVOS], float velocidad_deg_por_s = 60.0f, int paso_deg = 1) {
  int maxdiff = 0;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int d = abs(limitar(i + 1, objetivos[i]) - mov[i].actual);
    if (d > maxdiff) maxdiff = d;
  }
  if (maxdiff == 0) return;
  for (int i = 0; i < NUM_SERVOS; i++) {
    int d = abs(limitar(i + 1, objetivos[i]) - mov[i].actual);
    if (d == 0) continue;
    moverServoSuave(i + 1, objetivos[i], velocidad_deg_por_s * d / maxdiff, paso_deg);
  }
}

void actualizarMovimientos() {
  uint32_t ahora = micros();
  for (int i = 0; i < NUM_SERVOS; i++) {
    Movimiento &m = mov[i];
    if (m.actual == m.objetivo) continue;
    if ((uint32_t)(ahora - m.ultimo_us) < m.intervalo_us) continue;
    m.ultimo_us = ahora;
    int diff = m.objetivo - m.actual;
    int paso = min(m.paso, abs(diff));
    m.actual += diff > 0 ? paso : -paso;
    escribirServo(i + 1, m.actual);
  }
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
// CONTROL POR TECLAS (ahora llegan por /brazo/tecla)
// =========================================================
void procesarChar(char ch) {
  int servo = 0, delta = 0;
  switch (ch) {
    case 'w': servo = 1; delta = +STEP; break;
    case 's': servo = 1; delta = -STEP; break;
    case 'a': servo = 2; delta = -STEP; break;
    case 'd': servo = 2; delta = +STEP; break;
    case 'q': servo = 3; delta = +STEP; break;
    case 'e': servo = 3; delta = -STEP; break;
    case 'z': servo = 4; delta = -STEP; break;
    case 'c': servo = 4; delta = +STEP; break;
    case 'r': servo = 5; delta = +STEP; break;
    case 'f': servo = 5; delta = -STEP; break;
    case ' ':
      grip_closed = !grip_closed;
      moverServoSuave(6, grip_closed ? GRIP_CLOSE_ANG : GRIP_OPEN_ANG, VEL, PASO);
      return;
    default:
      return;
  }
  // Se suma sobre el objetivo para que pulsaciones seguidas se acumulen
  // igual que con el movimiento bloqueante de MicroPython.
  int nuevo = mov[servo - 1].objetivo + delta;
  if (servo == 1) nuevo = anguloSeguroS1(nuevo, delta);
  moverServoSuave(servo, nuevo, VEL, PASO);
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
  if (dist < DIST_THRESH_MM && mov[5].objetivo != GRIP_OPEN_ANG) {
    moverServoSuave(6, GRIP_OPEN_ANG, VEL, PASO);
    grip_closed = false;
  }
}

// Mientras el eje este inclinado mas alla de ZONA_MUERTA, el servo avanza.
// Se suma sobre el objetivo (no sobre la posicion actual) para que el avance
// se acumule muestra a muestra mientras dure la inclinacion.
void moverPorInclinacion(int32_t acc, int servo, int signo) {
  if (acc <= ZONA_MUERTA && acc >= -ZONA_MUERTA) return;
  int dir = (acc > 0 ? +1 : -1) * signo;
  int nuevo = mov[servo - 1].objetivo + dir * MPU_PASO;
  if (servo == 1) nuevo = anguloSeguroS1(nuevo, dir);  // FIX 8
  moverServoSuave(servo, nuevo, VEL, PASO);
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

rcl_subscription_t sub_tecla, sub_servo, sub_garra, sub_activar;
rcl_publisher_t pub_estado, pub_distancia, pub_imu, pub_diag;

std_msgs__msg__String msg_tecla;
char buf_tecla[32];
std_msgs__msg__Int32MultiArray msg_servo;
int32_t buf_servo[8];
std_msgs__msg__Bool msg_garra;
std_msgs__msg__Int32MultiArray msg_activar;
int32_t buf_activar[8];

std_msgs__msg__Int32MultiArray msg_estado;
int32_t buf_estado[NUM_SERVOS];
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

void cbServo(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *m = (const std_msgs__msg__Int32MultiArray *)msgin;
  const int32_t *d = m->data.data;
  if (m->data.size == 2 || m->data.size == 3) {
    int servo = d[0];
    if (servo < 1 || servo > NUM_SERVOS) return;
    int angulo = d[1];
    float vel = (m->data.size == 3 && d[2] > 0) ? (float)d[2] : VEL;
    if (servo == 1) angulo = anguloSeguroS1(angulo, angulo - mov[0].objetivo);
    if (servo == 6) grip_closed = angulo != GRIP_OPEN_ANG;
    moverServoSuave(servo, angulo, vel, PASO);
  } else if (m->data.size == NUM_SERVOS) {
    int objetivos[NUM_SERVOS];
    for (int i = 0; i < NUM_SERVOS; i++) objetivos[i] = d[i];
    objetivos[0] = anguloSeguroS1(objetivos[0], objetivos[0] - mov[0].objetivo);
    grip_closed = objetivos[5] != GRIP_OPEN_ANG;
    moverTodosSuave(objetivos, VEL, PASO);
  }
}

void cbGarra(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  grip_closed = m->data;
  moverServoSuave(6, grip_closed ? GRIP_CLOSE_ANG : GRIP_OPEN_ANG, VEL, PASO);
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
  msg_estado.data.capacity = NUM_SERVOS;
  msg_estado.data.size = NUM_SERVOS;

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
  sub_servo = rcl_get_zero_initialized_subscription();
  sub_garra = rcl_get_zero_initialized_subscription();
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
  RCCHECK(rclc_subscription_init_default(&sub_servo, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "brazo/servo"));
  RCCHECK(rclc_subscription_init_default(&sub_garra, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "brazo/garra"));
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
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_servo, &msg_servo, &cbServo, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_garra, &msg_garra, &cbGarra, ON_NEW_DATA));
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
  (void)rcl_subscription_fini(&sub_servo, &node);
  (void)rcl_subscription_fini(&sub_garra, &node);
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
    for (int i = 0; i < NUM_SERVOS; i++) buf_estado[i] = mov[i].actual;
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
    int n = snprintf(buf_diag, sizeof(buf_diag), "i2c: %s | tof: %s | mpu: %s | servos activos: %s",
                     i2c_encontrados, tof_ok ? "ok" : "no", mpu_ok ? "ok" : "no", activos);
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
  // Posicion inicial al centro de sus limites (equivale a init_pos()).
  // Los pulsos empiezan servo por servo desde arranqueEscalonado().
  for (int s = 1; s <= NUM_SERVOS; s++) {
    iniciarServo(s);
    const ConfigServo &c = cfg(s);
    int centro = (c.lim_min + c.lim_max) / 2;
    mov[s - 1] = {centro, centro, PASO, 0, 0};
  }

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
  reintentarSensores(ahora);
  muestrearTof(ahora);
  muestrearMpu(ahora);
}
