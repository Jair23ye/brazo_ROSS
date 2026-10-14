#!/usr/bin/env python3
# teleop_brazo.py -- control por teclado del brazo via ROS 2 (Linux / WSL)
# Replica el control serie de Thonny: cada tecla se publica en /brazo/tecla
# y el ESP32 la procesa con la misma tabla de teclas.
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
  w / s : levantar / bajar (servo 2)   a / d : girar la base (servo 1)
  q / e : servo 3 +/-                  z / c : servo 4 -/+
  r / f : servo 5 +/-                  espacio : abrir/cerrar garra
"""

TECLAS = set("wsadqezcrf ")

# Tecla del teleop -> tecla que entiende el firmware (tabla original de Thonny,
# donde w/s movian el servo 1 y a/d el servo 2). En el brazo real el servo 2
# sube/baja y el servo 1 gira la base, asi que se intercambian.
REMAPEO = {"w": "a", "s": "d", "a": "w", "d": "s"}


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
            print("\rS1-S6: {}      ".format(angulos), end="", flush=True)

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
                    pub.publish(String(data=REMAPEO.get(ch, ch)))
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
