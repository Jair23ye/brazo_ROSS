#!/usr/bin/env python3
# mando_xinput.py -- lee un mando Xbox en Windows (XInput) e imprime su estado.
# Lo lanza teleop_mando.py desde WSL; usa XInput1_4.dll de Windows, sin instalar nada.
# Salida: una linea por lectura, "lx ly rx ry lt rt botones", o "nc" si no hay mando.

import ctypes
import time

PERIODO_S = 1 / 50


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [("wButtons", ctypes.c_ushort),
                ("bLeftTrigger", ctypes.c_ubyte),
                ("bRightTrigger", ctypes.c_ubyte),
                ("sThumbLX", ctypes.c_short),
                ("sThumbLY", ctypes.c_short),
                ("sThumbRX", ctypes.c_short),
                ("sThumbRY", ctypes.c_short)]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_uint),
                ("Gamepad", XINPUT_GAMEPAD)]


def cargar_xinput():
    for nombre in ("XInput1_4.dll", "XInput9_1_0.dll", "XInput1_3.dll"):
        try:
            return ctypes.windll.LoadLibrary(nombre)
        except OSError:
            pass
    raise SystemExit("No se encontro XInput en Windows")


def main():
    xinput = cargar_xinput()
    estado = XINPUT_STATE()
    while True:
        # Usa el primer mando conectado (ranuras 0 a 3)
        linea = "nc"
        for indice in range(4):
            if xinput.XInputGetState(indice, ctypes.byref(estado)) == 0:
                g = estado.Gamepad
                linea = "%d %d %d %d %d %d %d" % (g.sThumbLX, g.sThumbLY, g.sThumbRX, g.sThumbRY,
                                                  g.bLeftTrigger, g.bRightTrigger, g.wButtons)
                break
        try:
            print(linea, flush=True)
        except (BrokenPipeError, OSError):
            return  # teleop_mando.py se cerro
        time.sleep(PERIODO_S)


if __name__ == "__main__":
    main()
