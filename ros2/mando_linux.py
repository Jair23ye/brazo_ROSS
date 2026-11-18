#!/usr/bin/env python3
# mando_linux.py -- lee un mando Xbox en Linux (evdev, /dev/input) e imprime su estado.
# Version Linux de mando_xinput.py: misma salida, asi teleop_mando.py no cambia su logica.
# Solo usa la biblioteca estandar (sin pip install).
# Salida: una linea por lectura, "lx ly rx ry lt rt botones", o "nc" si no hay mando.
# Rangos como XInput: sticks -32768..32767 (arriba = positivo), gatillos 0..255,
# botones con las mismas mascaras (A = 0x1000).
#
# Prueba sola, sin ROS:  python3 mando_linux.py

import fcntl
import glob
import os
import re
import struct
import sys
import time

PERIODO_S = 1 / 50
REINTENTO_S = 1.0

EV_SYN, EV_KEY, EV_ABS = 0x00, 0x01, 0x03
SYN_DROPPED = 3
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_GAS, ABS_BRAKE = 0x09, 0x0A
BTN_SOUTH = 0x130

# Boton de Linux -> mascara de XInput
BOTONES = {
    0x130: 0x1000,  # A
    0x131: 0x2000,  # B
    0x133: 0x4000,  # X
    0x134: 0x8000,  # Y
    0x136: 0x0100,  # LB
    0x137: 0x0200,  # RB
    0x13A: 0x0020,  # Back / View
    0x13B: 0x0010,  # Start / Menu
}

EVENTO = struct.Struct("llHHi")  # struct input_event (timeval, type, code, value)
ABSINFO = struct.Struct("6i")    # value, minimum, maximum, fuzz, flat, resolution


def _ioc_leer(nr, tamano):
    # _IOR('E', nr, tamano) de <linux/input.h>
    return (2 << 30) | (tamano << 16) | (ord("E") << 8) | nr


def _tiene(bits, codigo):
    return bool((bits >> codigo) & 1)


def _capacidades(evento, tipo):
    # /sys/class/input/eventN/device/capabilities/{key,abs}: palabras hex de
    # tamano long, la mas significativa primero. Se lee sin permisos especiales.
    try:
        with open("/sys/class/input/%s/device/capabilities/%s" % (evento, tipo)) as f:
            palabras = f.read().split()
    except OSError:
        return 0
    bits_por_palabra = struct.calcsize("l") * 8
    bits = 0
    for palabra in palabras:
        bits = (bits << bits_por_palabra) | int(palabra, 16)
    return bits


def a_rango(valor, minimo, maximo, lo, hi):
    if maximo == minimo:
        return 0
    f = (valor - minimo) / (maximo - minimo)
    return max(lo, min(hi, int(round(lo + f * (hi - lo)))))


class Mando:
    def __init__(self, fd, abs_bits):
        self.fd = fd
        # Con xpad/xpadneo el stick derecho es RX/RY y los gatillos Z/RZ.
        # Por Bluetooth sin xpadneo el stick derecho es Z/RZ y los gatillos BRAKE/GAS.
        if _tiene(abs_bits, ABS_RX) and _tiene(abs_bits, ABS_RY):
            self.ejes = {"lx": ABS_X, "ly": ABS_Y, "rx": ABS_RX, "ry": ABS_RY, "lt": ABS_Z, "rt": ABS_RZ}
        else:
            self.ejes = {"lx": ABS_X, "ly": ABS_Y, "rx": ABS_Z, "ry": ABS_RZ, "lt": ABS_BRAKE, "rt": ABS_GAS}
        self.rangos = {}
        self.crudo = {}
        self.botones = 0
        self.leer_ejes()

    def leer_ejes(self):
        # Valor actual de cada eje (al conectar o si el kernel descarto eventos)
        for codigo in self.ejes.values():
            buf = bytearray(ABSINFO.size)
            try:
                fcntl.ioctl(self.fd, _ioc_leer(0x40 + codigo, ABSINFO.size), buf)
            except OSError:
                continue  # el mando no tiene ese eje
            valor, minimo, maximo = ABSINFO.unpack(buf)[:3]
            self.rangos[codigo] = (minimo, maximo)
            self.crudo[codigo] = valor

    def procesar_eventos(self):
        # Lanza OSError si el mando se desconecto
        while True:
            try:
                datos = os.read(self.fd, EVENTO.size * 64)
            except BlockingIOError:
                return
            for i in range(0, len(datos) - EVENTO.size + 1, EVENTO.size):
                _, _, tipo, codigo, valor = EVENTO.unpack_from(datos, i)
                if tipo == EV_ABS and codigo in self.rangos:
                    self.crudo[codigo] = valor
                elif tipo == EV_KEY and codigo in BOTONES:
                    if valor:
                        self.botones |= BOTONES[codigo]
                    else:
                        self.botones &= ~BOTONES[codigo]
                elif tipo == EV_SYN and codigo == SYN_DROPPED:
                    self.leer_ejes()

    def eje(self, nombre, lo, hi, invertir=False):
        codigo = self.ejes[nombre]
        if codigo not in self.rangos:
            return 0
        minimo, maximo = self.rangos[codigo]
        v = a_rango(self.crudo[codigo], minimo, maximo, lo, hi)
        # En Linux el stick hacia arriba es negativo; en XInput es positivo
        return min(hi, -v) if invertir else v

    def linea(self):
        return "%d %d %d %d %d %d %d" % (
            self.eje("lx", -32768, 32767),
            self.eje("ly", -32768, 32767, invertir=True),
            self.eje("rx", -32768, 32767),
            self.eje("ry", -32768, 32767, invertir=True),
            self.eje("lt", 0, 255),
            self.eje("rt", 0, 255),
            self.botones,
        )

    def cerrar(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def buscar_mando():
    """Primer /dev/input/eventN con boton A y stick izquierdo.
    Devuelve (Mando o None, ruta del mando que no se pudo abrir por permisos o None)."""
    rutas = sorted(glob.glob("/dev/input/event*"), key=lambda r: int(re.sub(r"\D", "", r) or 0))
    for ruta in rutas:
        evento = os.path.basename(ruta)
        abs_bits = _capacidades(evento, "abs")
        if not (_tiene(_capacidades(evento, "key"), BTN_SOUTH)
                and _tiene(abs_bits, ABS_X) and _tiene(abs_bits, ABS_Y)):
            continue
        try:
            fd = os.open(ruta, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError:
            return None, ruta
        except OSError:
            continue
        return Mando(fd, abs_bits), None
    return None, None


def main():
    mando = None
    avisado = False
    ultimo_intento = -REINTENTO_S
    siguiente = time.monotonic()
    while True:
        ahora = time.monotonic()
        if mando is None and ahora - ultimo_intento >= REINTENTO_S:
            ultimo_intento = ahora
            mando, sin_permiso = buscar_mando()
            if sin_permiso and not avisado:
                print("\nmando_linux: sin permiso para leer el mando en %s "
                      "(agrega tu usuario al grupo input)" % sin_permiso, file=sys.stderr, flush=True)
                avisado = True
        if mando is not None:
            try:
                mando.procesar_eventos()
            except OSError:
                mando.cerrar()  # se desconecto: se vuelve a buscar
                mando = None
        try:
            print(mando.linea() if mando else "nc", flush=True)
        except (BrokenPipeError, OSError):
            return  # teleop_mando.py se cerro
        siguiente += PERIODO_S
        time.sleep(max(0.0, siguiente - time.monotonic()))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
