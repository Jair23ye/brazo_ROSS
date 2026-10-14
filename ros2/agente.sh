#!/bin/bash
# Arranca el agente micro-ROS para el brazo (ESP32 por USB en WSL)
# Uso: bash agente.sh [/dev/ttyACM0]
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
# Si un agente anterior se cerro a la fuerza, deja memoria compartida huerfana
# y los nodos nuevos dejan de recibir datos sin dar error. Solo borra lo no usado.
fastdds shm clean > /dev/null 2>&1
# El agente publica solo por UDP: asi cualquier nodo recibe sus datos
# aunque la memoria compartida de Fast DDS falle.
export FASTRTPS_DEFAULT_PROFILES_FILE="$(dirname "$(readlink -f "$0")")/fastdds_udp.xml"
ros2 run micro_ros_agent micro_ros_agent serial --dev "${1:-/dev/ttyACM0}" -b 115200 -v4
