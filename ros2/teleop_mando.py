#!/usr/bin/env python3
# teleop_mando.py -- control del brazo con un mando Xbox via ROS 2 (Linux)
# El mando se lee de /dev/input con mando_linux.py y aqui se traduce:
#   - Servos: a las mismas teclas que entiende el firmware en /brazo/tecla
#     (2 grados por tecla). Mientras mas se inclina el stick, mas teclas por
#     segundo: mas velocidad.
#   - Base (NEMA 23): velocidad en % por /brazo/base, proporcional al stick.
#     Se repite mientras el stick este inclinado; si deja de llegar, el ESP32 frena.
#
# Uso:  python3 teleop_mando.py      (o, sin ROS 2 instalado: bash teleop.sh mando)

import math
import os
import subprocess
import sys
import time

# Fast DDS solo por UDP: en este WSL la memoria compartida deja de entregar datos.
# Debe fijarse antes de crear el nodo.
DIR = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", os.path.join(DIR, "fastdds_udp.xml"))

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32, Int32MultiArray, String

AYUDA = """
Control del brazo con mando Xbox (Ctrl+C para salir)
  Stick izquierdo a los lados  : girar la base (NEMA 23)
  Stick izquierdo arriba/abajo : levantar / bajar (servo 2)
  Stick derecho arriba/abajo   : servo 3
  Stick derecho a los lados    : servo 4
  RB / LB                      : servo 1 +/-
  Boton Y                      : soltar / trabar la base (suelta: libre para moverla a mano)
  Gatillos RT / LT             : cerrar / abrir garra (poco a poco)
  Mas inclinacion del stick = mas velocidad
"""

ZONA_STICK = 8000      # de 32767; evita que el stick en reposo mueva el brazo
ZONA_GATILLO = 40      # de 255
TECLAS_S_MIN = 3.0     # 6 grados/s con el stick apenas inclinado
TECLAS_S_MAX = 20.0    # 40 grados/s con el stick a fondo
TECLAS_S_BOTON = 8.0   # RB/LB: 16 grados/s

# Base: % de la velocidad maxima del firmware (VEL_BASE_MAX). Curva cuadratica:
# cerca del centro el stick da velocidades bajas para acomodar la base fino.
BASE_PCT_MIN = 3
BASE_REPETIR_S = 0.1   # mientras gire, se repite la orden (el ESP32 frena a los 0.35 s sin ordenes)

BOTON_LB, BOTON_RB, BOTON_Y = 0x0100, 0x0200, 0x8000

# (eje, tecla del firmware si es positivo, tecla si es negativo, zona muerta, maximo).
# Firmware: w/s servo 2 (w sube), t/g servo 1, q/e servo 3, z/c servo 4, r/f garra (r cierra).
EJES = [
    ("ly", "w", "s", ZONA_STICK, 32767),
    ("ry", "q", "e", ZONA_STICK, 32767),
    ("rx", "c", "z", ZONA_STICK, 32767),
    ("rt", "r", None, ZONA_GATILLO, 255),
    ("lt", "f", None, ZONA_GATILLO, 255),
]
BOTONES = [(BOTON_RB, "t"), (BOTON_LB, "g")]


def fraccion(valor, zona, maximo):
    v = abs(valor)
    if v <= zona:
        return 0.0
    return min(1.0, (v - zona) / (maximo - zona))


def teclas_por_segundo(valor, zona, maximo):
    f = fraccion(valor, zona, maximo)
    return 0.0 if f == 0.0 else TECLAS_S_MIN + (TECLAS_S_MAX - TECLAS_S_MIN) * f


def porcentaje_base(lx):
    f = fraccion(lx, ZONA_STICK, 32767)
    if f == 0.0:
        return 0
    return int(math.copysign(round(BASE_PCT_MIN + (100 - BASE_PCT_MIN) * f * f), lx))


def texto_estado(angulos):
    # /brazo/estado = [s1..s5, base]; un firmware anterior manda solo los servos
    if len(angulos) == 6:
        return "Base: {:5d} grados   S1-S5: {}".format(angulos[5], angulos[:5])
    return "S1-S5: {}".format(angulos)


def lanzar_lector():
    # stderr se deja pasar: ahi avisa si no tiene permiso para leer el mando
    return subprocess.Popen([sys.executable, "-u", os.path.join(DIR, "mando_linux.py")],
                            stdout=subprocess.PIPE, text=True, bufsize=1)


class Repetidor:
    """Convierte una tasa (teclas/s) en teclas: la primera sale al momento y luego
    se acumula la fraccion de tecla de cada lectura."""

    def __init__(self):
        self.acumulado = 0.0

    def teclas(self, tasa, tecla, dt):
        if tasa == 0.0:
            self.acumulado = 0.0
            return ""
        if self.acumulado == 0.0:
            self.acumulado = 1.0  # primera tecla inmediata al mover el stick
        else:
            self.acumulado += tasa * dt
        cuantas = int(self.acumulado)
        self.acumulado = max(self.acumulado - cuantas, 1e-6)  # sigue "en movimiento"
        return tecla * cuantas

    def soltar(self):
        self.acumulado = 0.0


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_mando")
    pub = node.create_publisher(String, "brazo/tecla", 10)
    pub_base = node.create_publisher(Int32, "brazo/base", 10)
    estado = {"angulos": None}

    def cb_estado(msg):
        angulos = list(msg.data)
        if angulos != estado["angulos"]:
            if estado["angulos"] is None:
                print("ESP32 conectado.")
            estado["angulos"] = angulos
            print("\r" + texto_estado(angulos) + "      ", end="", flush=True)

    node.create_subscription(Int32MultiArray, "brazo/estado", cb_estado, qos_profile_sensor_data)
    print(AYUDA)
    print("Esperando al ESP32 (debe estar corriendo el agente)...")

    lector = lanzar_lector()
    repetidores = {nombre: Repetidor() for nombre in [e[0] for e in EJES] + [b[1] for b in BOTONES]}
    base = {"pct": 0, "enviado": 0.0}
    botones_antes = 0
    avisado = {"t": -10.0}
    mando_conectado = None
    ultimo = time.monotonic()

    def mandar_base(pct, ahora):
        # Al cambiar se manda al momento; girando, se repite cada BASE_REPETIR_S
        if pct != base["pct"] or (pct != 0 and ahora - base["enviado"] >= BASE_REPETIR_S):
            pub_base.publish(Int32(data=pct))
            base["pct"], base["enviado"] = pct, ahora

    try:
        while rclpy.ok():
            linea = lector.stdout.readline()
            if not linea:
                print("\nSe cerro el lector del mando.")
                break
            ahora = time.monotonic()
            dt = min(ahora - ultimo, 0.1)
            ultimo = ahora

            partes = linea.split()
            conectado = partes != ["nc"]
            if conectado != mando_conectado:
                print("\nMando conectado." if conectado else "\nNo hay mando conectado.")
                mando_conectado = conectado
            if not conectado or len(partes) != 7:
                for r in repetidores.values():
                    r.soltar()
                mandar_base(0, ahora)
                rclpy.spin_once(node, timeout_sec=0)
                continue

            lx, ly, rx, ry, lt, rt, botones = map(int, partes)
            apretados = botones & ~botones_antes  # solo el momento en que se aprieta
            botones_antes = botones
            valores = {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "lt": lt, "rt": rt}

            mandar_base(porcentaje_base(lx), ahora)

            teclas = "x" if apretados & BOTON_Y else ""
            for eje, positiva, negativa, zona, maximo in EJES:
                valor = valores[eje]
                tecla = positiva if valor > 0 else negativa
                teclas += repetidores[eje].teclas(teclas_por_segundo(valor, zona, maximo), tecla, dt)
            for mascara, tecla in BOTONES:
                tasa = TECLAS_S_BOTON if botones & mascara else 0.0
                teclas += repetidores[tecla].teclas(tasa, tecla, dt)

            if (teclas or base["pct"] != 0) and ahora - avisado["t"] > 2.0:
                if pub.get_subscription_count() == 0:
                    print("\nEl ESP32 no esta conectado al agente: el mando no llega.")
                    avisado["t"] = ahora
            if teclas:
                pub.publish(String(data=teclas))
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        lector.terminate()
        if rclpy.ok():
            pub_base.publish(Int32(data=0))  # que la base no siga girando
        print()
        node.destroy_node()
        if rclpy.ok():  # con Ctrl+C, rclpy ya cerro el contexto
            rclpy.shutdown()


if __name__ == "__main__":
    main()
