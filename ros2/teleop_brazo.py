#!/usr/bin/env python3
# teleop_brazo.py -- control por teclado del brazo via ROS 2 (Linux / WSL)
# Cada tecla se publica en /brazo/tecla y el ESP32 la procesa con la misma tabla.
# La base (NEMA 23) gira mientras se mantiene a/d: el teclado repite la tecla y el
# ESP32 frena la base cuando deja de llegar.
# Muestra los angulos que reporta el ESP32 en /brazo/estado.
#
# Uso:  source /opt/ros/humble/setup.bash && python3 teleop_brazo.py

import os
import select
import sys
import termios
import tty

# Fast DDS solo por UDP: en este WSL la memoria compartida deja de entregar datos.
# Debe fijarse antes de crear el nodo.
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE",
                      os.path.join(os.path.dirname(os.path.abspath(__file__)), "fastdds_udp.xml"))

import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray, String

AYUDA = """
Control del brazo (Ctrl+C para salir)
  a / d : girar la base (NEMA 23) mientras mantengas la tecla
  x     : soltar / trabar la base (suelta: libre para moverla a mano)
  w / s : levantar / bajar (servo 2)   t / g : servo 1 +/-
  q / e : servo 3 +/-                  z / c : servo 4 -/+
  r / f : cerrar / abrir garra (poco a poco)
"""

# El firmware usa estas mismas letras (ya no hace falta remapear como con Thonny)
TECLAS = set("adxwstgqezcrf")


def texto_estado(angulos):
    # /brazo/estado = [s1..s5, base]; un firmware anterior manda solo los servos
    if len(angulos) == 6:
        return "Base: {:5d} grados   S1-S5: {}".format(angulos[5], angulos[:5])
    return "S1-S5: {}".format(angulos)


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_brazo")
    pub = node.create_publisher(String, "brazo/tecla", 10)
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

    viejo = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1).lower()
                if ch in TECLAS:
                    if pub.get_subscription_count() == 0:
                        print("\nEl ESP32 no esta conectado al agente: la tecla no llega.")
                    pub.publish(String(data=ch))
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        print()
        node.destroy_node()
        if rclpy.ok():  # con Ctrl+C, rclpy ya cerro el contexto
            rclpy.shutdown()


if __name__ == "__main__":
    main()
