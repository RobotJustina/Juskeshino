#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import numpy as np
import time 
import sys


ADDRESS = 0x80
speed = 127
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()
iteraciones = 20

def calculate_qpps(duration=5.0):

    roboclaw.ForwardM1(ADDRESS, 0)
    roboclaw.ForwardM2(ADDRESS, 0)
    time.sleep(2.0)
    _,enc1_init,__ = roboclaw.ReadEncM1(ADDRESS)
    _,enc2_init,__ = roboclaw.ReadEncM2(ADDRESS)
    roboclaw.ForwardM1(ADDRESS, speed)
    roboclaw.ForwardM2(ADDRESS, speed)
    time.sleep(duration)
    _,enc1_end,__ = roboclaw.ReadEncM1(ADDRESS)
    _,enc2_end,__ = roboclaw.ReadEncM2(ADDRESS)
    roboclaw.ForwardM1(ADDRESS, 0)
    roboclaw.ForwardM2(ADDRESS, 0)
    qpps1= (enc1_end - enc1_init)/ duration
    qpps2= (enc2_end - enc2_init)/ duration
    print ("qpps1:", qpps1)
    print ("qpps2:", qpps2)
    return qpps1, qpps2


def get_mean_qpps(iterations, duration):
    qpps1s = []
    qpps2s = []
    for i in range(iterations):
        qpps1, qpps2 = calculate_qpps(duration)
        qpps1s.append(qpps1)
        qpps2s.append(qpps2)
    qpps1s = np.asarray(qpps1s)
    qpps2s = np.asarray(qpps2s)
    print("QPPS1:", np.mean(qpps1s), np.std(qpps1s))
    print("QPPS2:", np.mean(qpps2s), np.std(qpps2s))


def move_motor(duracion=iteraciones):

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
    try:
        duration=float(sys.argv[1])
    except:
        duration=5.0

    get_mean_qpps(4, duration)

