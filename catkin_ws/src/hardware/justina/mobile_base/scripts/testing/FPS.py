#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import numpy as np
import time

adress = 0x80
vel = 127
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()
iteracciones = 20
def mover_motor(duracion=iteracciones):

    inicio = time.time()
    tiempo_lec = []
    enc1_val = []
    enc2_val = []

    roboclaw.ForwardM1(adress, vel)
    for _ in range(50): 
        leer_encoders(enc1_val, enc2_val, tiempo_lec)
        time.sleep(0.1)  
    roboclaw.BackwardM1(adress, vel)
    for _ in range(50):
        leer_encoders(enc1_val, enc2_val, tiempo_lec)
        time.sleep(0.1)
    roboclaw.ForwardM1(adress, 0)


    roboclaw.ForwardM2(adress, vel)
    for _ in range(50):
        leer_encoders(enc1_val, enc2_val, tiempo_lec)
        time.sleep(0.1)
    roboclaw.BackwardM2(adress, vel)
    for _ in range(50):
        leer_encoders(enc1_val, enc2_val, tiempo_lec)
        time.sleep(0.1)
    roboclaw.ForwardM2(adress, 0)

    calcular_fps(tiempo_lec)

def leer_encoders(enc1_val, enc2_val, tiempo_lec):
    t_inicio = time.time()
    try:
        enc1, status1, _ = roboclaw.ReadEncM1(adress)
        enc2, status2, _ = roboclaw.ReadEncM2(adress)

        if status1 and status2:
            enc1_val.append(enc1)
            enc2_val.append(enc2)
            tiempo_lec.append(time.time() - t_inicio)
        else:
            print("Error en la lectura de encoders")

    except Exception as e:
        print(f"Error en la lectura: {e}")

def calcular_fps(tiempo_lec):
    """Calcula y muestra FPS basado en la media y desviación estándar del tiempo entre lecturas"""
    if tiempo_lec:
        tiempo_lec = np.array(tiempo_lec)
        tiempo_medio = np.mean(tiempo_lec)
        desviacion_tiempo = np.std(tiempo_lec)

        fps_medio = 1 / tiempo_medio if tiempo_medio > 0 else 0
        fps_min = 1 / (tiempo_medio + desviacion_tiempo) if (tiempo_medio + desviacion_tiempo) > 0 else 0
        fps_max = 1 / (tiempo_medio - desviacion_tiempo) if (tiempo_medio - desviacion_tiempo) > 0 else fps_medio

        print(f"FPS medio: {fps_medio:.2f} (mín: {fps_min:.2f}, máx: {fps_max:.2f})")
    else:
        print("No se pudieron obtener lecturas válidas")

if __name__ == "__main__":
    mover_motor()

