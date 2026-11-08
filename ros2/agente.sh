#!/bin/bash
# Arranca el agente micro-ROS para el brazo (ESP32 por USB en WSL)
# Uso: bash agente.sh [/dev/ttyACM0]
DEV="${1:-/dev/ttyACM0}"

# Si el ESP32 no esta en WSL, lo pasa con usbipd de Windows (no requiere admin).
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
if [ ! -e "$DEV" ] && [ -x "$USBIPD" ]; then
  LINEA=$("$USBIPD" list 2>/dev/null | tr -d '\r' | grep "1a86:55d4" | head -1)
  BUSID=$(echo "$LINEA" | awk '{print $1}')
  if [ -z "$BUSID" ]; then
    echo ">> No veo el ESP32 conectado a la laptop. Revisa el cable USB."
  elif echo "$LINEA" | grep -q "Not shared"; then
    echo ">> El ESP32 (BUSID $BUSID) no esta compartido. En PowerShell como administrador:"
    echo "     usbipd bind --busid $BUSID"
  else
    echo ">> Pasando el ESP32 (BUSID $BUSID) a WSL..."
    "$USBIPD" attach --wsl --busid "$BUSID" 2>&1 | tr -d '\r' | grep -v "info:"
    for _ in $(seq 10); do [ -e "$DEV" ] && break; sleep 1; done
  fi
  [ -e "$DEV" ] && echo ">> Listo: $DEV" || echo ">> $DEV no aparecio; el agente esperara a que se conecte."
fi

source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash
# Si un agente anterior se cerro a la fuerza, deja memoria compartida huerfana
# y los nodos nuevos dejan de recibir datos sin dar error. Solo borra lo no usado.
fastdds shm clean > /dev/null 2>&1
# El agente publica solo por UDP: asi cualquier nodo recibe sus datos
# aunque la memoria compartida de Fast DDS falle.
export FASTRTPS_DEFAULT_PROFILES_FILE="$(dirname "$(readlink -f "$0")")/fastdds_udp.xml"
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b 115200 -v4
