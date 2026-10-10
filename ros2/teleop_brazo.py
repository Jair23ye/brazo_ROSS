#!/usr/bin/env python3
# teleop_brazo.py -- control por teclado del brazo via ROS 2 (Linux / WSL)
# Replica el control serie de Thonny: cada tecla se publica en /brazo/tecla
# y el ESP32 la procesa con la misma tabla de teclas.
#
# Uso:  source /opt/ros/humble/setup.bash && python3 teleop_brazo.py

import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import String

AYUDA = """
Control del brazo (Ctrl+C para salir)
  w / s : servo 1 +/-     a / d : servo 2 -/+
  q / e : servo 3 +/-     z / c : servo 4 -/+
  r / f : servo 5 +/-     espacio : abrir/cerrar garra
"""

TECLAS = set("wsadqezcrf ")


def main():
    rclpy.init()
    node = rclpy.create_node("teleop_brazo")
    pub = node.create_publisher(String, "brazo/tecla", 10)
    print(AYUDA)

    viejo = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1).lower()
                if ch in TECLAS:
                    pub.publish(String(data=ch))
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
