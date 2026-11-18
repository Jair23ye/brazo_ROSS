#!/bin/bash
# Control del brazo por ROS 2:
#   teclado -> teleop_brazo.py      mando -> teleop_mando.py
#   shell   -> terminal con ROS 2 lista (ros2 topic echo/pub ...)
# Con ROS 2 Humble instalado lo corre directo; si no, en un contenedor Docker.
# Uso: bash teleop.sh [teclado|mando|shell]
DIR="$(dirname "$(readlink -f "$0")")"
case "${1:-teclado}" in
  teclado) ARGS=(python3 teleop_brazo.py) ;;
  mando)   ARGS=(python3 teleop_mando.py) ;;
  shell)   ARGS=(bash) ;;
  *) echo "Uso: bash teleop.sh [teclado|mando|shell]"; exit 1 ;;
esac

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DIR/fastdds_udp.xml"
  cd "$DIR" && exec "${ARGS[@]}"
fi

if ! docker info > /dev/null 2>&1; then
  echo ">> No hay ROS 2 Humble y Docker no esta disponible. Una sola vez:"
  echo "     sudo usermod -aG docker \$USER    (luego cerrar sesion y volver a entrar)"
  echo ">> Si ya lo hiciste y no has cerrado sesion: corre 'newgrp docker' en esta terminal."
  exit 1
fi
# --net=host: ve al agente por UDP en la misma maquina. Mismo perfil solo-UDP.
# /dev/input + regla para los dispositivos de entrada (major 13): el mando se
# puede desconectar y volver a conectar sin reiniciar el contenedor.
exec docker run -it --rm --net=host \
  -v "$DIR:/ros2:ro" -w /ros2 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/ros2/fastdds_udp.xml \
  -v /dev/input:/dev/input:ro --device-cgroup-rule='c 13:* rmw' \
  ros:humble-ros-base "${ARGS[@]}"
