#!/usr/bin/env python3
# teleop_mando.py -- control del brazo con un mando Xbox via ROS 2 (Linux)
# El mando se lee de /dev/input con mando_linux.py y aqui se traduce a las
# mismas teclas que entiende el firmware en /brazo/tecla (2 grados por tecla).
# Mientras mas se inclina el stick, mas teclas por segundo: mas velocidad.
#
# Uso:  python3 teleop_mando.py      (o, sin ROS 2 instalado: bash teleop.sh mando)

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
from std_msgs.msg import Int32MultiArray, String

AYUDA = """
Control del brazo con mando Xbox (Ctrl+C para salir)
  Stick izquierdo arriba/abajo : levantar / bajar (servo 2)
  Stick izquierdo a los lados  : girar la base (servo 1)
  Stick derecho arriba/abajo   : servo 3
  Stick derecho a los lados    : servo 4
  Gatillos RT / LT             : servo 5 +/-
  Boton A                      : abrir / cerrar garra
  Mas inclinacion del stick = mas velocidad
"""

ZONA_STICK = 8000      # de 32767; evita que el stick en reposo mueva el brazo
ZONA_GATILLO = 40      # de 255
TECLAS_S_MIN = 3.0     # 6 grados/s con el stick apenas inclinado
TECLAS_S_MAX = 20.0    # 40 grados/s con el stick a fondo
BOTON_A = 0x1000

# (eje, tecla del firmware si es positivo, tecla si es negativo, zona muerta, maximo).
# Firmware: w/s servo 1, a/d servo 2 (a sube), q/e servo 3, z/c servo 4, r/f servo 5.
# Los sentidos de la base coinciden con a/d del teleop de teclado.
EJES = [
    ("ly", "a", "d", ZONA_STICK, 32767),
    ("lx", "s", "w", ZONA_STICK, 32767),
    ("ry", "q", "e", ZONA_STICK, 32767),
    ("rx", "c", "z", ZONA_STICK, 32767),
    ("rt", "r", None, ZONA_GATILLO, 255),
    ("lt", "f", None, ZONA_GATILLO, 255),
]


def teclas_por_segundo(valor, zona, maximo):
    v = abs(valor)
    if v <= zona:
        return 0.0
    fraccion = min(1.0, (v - zona) / (maximo - zona))
    return TECLAS_S_MIN + (TECLAS_S_MAX - TECLAS_S_MIN) * fraccion


def lanzar_lector():
    # stderr se deja pasar: ahi avisa si no tiene permiso para leer el mando
    return subprocess.Popen([sys.executable, "-u", os.path.join(DIR, "mando_linux.py")],
                            stdout=subprocess.PIPE, text=True, bufsize=1)


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_mando")
    pub = node.create_publisher(String, "brazo/tecla", 10)
    estado = {"angulos": None}

    def cb_estado(msg):
        angulos = list(msg.data)
        if angulos != estado["angulos"]:
            if estado["angulos"] is None:
                print("ESP32 conectado.")
            estado["angulos"] = angulos
            print("\rS1-S6: {}      ".format(angulos), end="", flush=True)

    node.create_subscription(Int32MultiArray, "brazo/estado", cb_estado, qos_profile_sensor_data)
    print(AYUDA)
    print("Esperando al ESP32 (debe estar corriendo el agente)...")

    lector = lanzar_lector()
    acumulado = {eje[0]: 0.0 for eje in EJES}
    botones_previos = 0
    mando_conectado = None
    ultimo = time.monotonic()
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
                acumulado = dict.fromkeys(acumulado, 0.0)
                rclpy.spin_once(node, timeout_sec=0)
                continue

            lx, ly, rx, ry, lt, rt, botones = map(int, partes)
            valores = {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "lt": lt, "rt": rt}

            teclas = ""
            for eje, positiva, negativa, zona, maximo in EJES:
                valor = valores[eje]
                tasa = teclas_por_segundo(valor, zona, maximo)
                if tasa == 0.0:
                    acumulado[eje] = 0.0
                    continue
                if acumulado[eje] == 0.0:
                    acumulado[eje] = 1.0  # primera tecla inmediata al mover el stick
                else:
                    acumulado[eje] += tasa * dt
                cuantas = int(acumulado[eje])
                acumulado[eje] -= cuantas
                tecla = positiva if valor > 0 else negativa
                teclas += tecla * cuantas
                acumulado[eje] = max(acumulado[eje], 1e-6)  # sigue "en movimiento"

            if botones & BOTON_A and not botones_previos & BOTON_A:
                teclas += " "
            botones_previos = botones

            if teclas:
                if pub.get_subscription_count() == 0:
                    print("\nEl ESP32 no esta conectado al agente: el mando no llega.")
                pub.publish(String(data=teclas))
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        lector.terminate()
        print()
        node.destroy_node()
        if rclpy.ok():  # con Ctrl+C, rclpy ya cerro el contexto
            rclpy.shutdown()


if __name__ == "__main__":
    main()
