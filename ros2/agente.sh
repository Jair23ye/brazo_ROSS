#!/bin/bash
# Arranca el agente micro-ROS para el brazo (ESP32 por USB en WSL)
# Uso: bash agente.sh [/dev/ttyACM0]
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev "${1:-/dev/ttyACM0}" -b 115200 -v4
