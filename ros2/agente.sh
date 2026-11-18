#!/bin/bash
# Arranca el agente micro-ROS para el brazo (ESP32 por USB).
#   - WSL: usa el ROS 2 Humble instalado y pasa el ESP32 con usbipd.
#   - Linux sin ROS 2 (Omarchy/Arch): corre el agente en Docker.
# Uso: bash agente.sh [/dev/ttyACM0]
DEV="${1:-/dev/ttyACM0}"
DIR="$(dirname "$(readlink -f "$0")")"

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

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  source ~/microros_ws/install/local_setup.bash
  # Si un agente anterior se cerro a la fuerza, deja memoria compartida huerfana
  # y los nodos nuevos dejan de recibir datos sin dar error. Solo borra lo no usado.
  fastdds shm clean > /dev/null 2>&1
  # El agente publica solo por UDP: asi cualquier nodo recibe sus datos
  # aunque la memoria compartida de Fast DDS falle.
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp.xml"
  exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b 115200 -v4
fi

# Sin ROS 2 instalado: agente en Docker. La imagen debe ser la misma distro
# que board_microros_distro en platformio.ini (humble).
if ! docker info > /dev/null 2>&1; then
  echo ">> No hay ROS 2 Humble y Docker no esta disponible. Una sola vez:"
  echo "     sudo usermod -aG docker,uucp \$USER    (luego cerrar sesion y volver a entrar)"
  echo ">> Si ya lo hiciste y no has cerrado sesion: corre 'newgrp docker' en esta terminal."
  exit 1
fi
[ -e "$DEV" ] || echo ">> $DEV no existe todavia; el agente esperara a que se conecte el ESP32."
# /dev completo + privileged: si el ESP32 se reconecta, el agente lo vuelve a ver.
# Mismo perfil solo-UDP que en WSL: el contenedor no comparte memoria con los
# nodos de fuera (MICROROS_DISABLE_SHM=0 evita que la imagen ponga el suyo).
exec docker run -it --rm --net=host -v /dev:/dev --privileged \
  -v "$DIR/fastdds_udp.xml:/fastdds_udp.xml:ro" \
  -e MICROROS_DISABLE_SHM=0 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp.xml \
  microros/micro-ros-agent:humble serial --dev "$DEV" -b 115200 -v4
