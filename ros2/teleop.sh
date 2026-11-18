#!/bin/bash
# Lanza el control del brazo: teclado (teleop_brazo.py) o mando Xbox (teleop_mando.py).
# Con ROS 2 Humble instalado lo corre directo; si no, en un contenedor Docker.
# Uso: bash teleop.sh [teclado|mando]
DIR="$(dirname "$(readlink -f "$0")")"
case "${1:-teclado}" in
  teclado) SCRIPT=teleop_brazo.py ;;
  mando)   SCRIPT=teleop_mando.py ;;
  *) echo "Uso: bash teleop.sh [teclado|mando]"; exit 1 ;;
esac

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  exec python3 "$DIR/$SCRIPT"
fi

if ! docker info > /dev/null 2>&1; then
  echo ">> No hay ROS 2 Humble y Docker no esta disponible. Una sola vez:"
  echo "     sudo systemctl enable --now docker"
  echo "     sudo usermod -aG docker \$USER    (luego cerrar sesion y volver a entrar)"
  exit 1
fi
# --net=host: ve al agente por UDP en la misma maquina.
# /dev/input + regla para los dispositivos de entrada (major 13): el mando se
# puede desconectar y volver a conectar sin reiniciar el contenedor.
exec docker run -it --rm --net=host \
  -v "$DIR:/ros2:ro" \
  -v /dev/input:/dev/input:ro --device-cgroup-rule='c 13:* rmw' \
  ros:humble-ros-base python3 "/ros2/$SCRIPT"
