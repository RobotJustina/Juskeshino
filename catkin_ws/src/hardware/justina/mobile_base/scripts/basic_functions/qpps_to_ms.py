#!/usr/bin/env python

from roboclaw_3 import Roboclaw
import time
import numpy as np 

ADDRESS = 0x80
qpps_1 = 1000
qpps_2 = 1000
qpps_3 = 1000
roboclaw = Roboclaw("/dev/ttyACM0", 38400)
roboclaw.Open()

# Parámetros físicos
CPR = 500  # estandar 
PPR = CPR * 4  #pulsos por revolucion 
D = 0.3 # diametro de la rueda 
C = np.pi * D  
R = 0.2  


M_inv = (D / 6) * np.array([
    [-1, -1, 2],
    [np.sqrt(3), -np.sqrt(3), 0],
    [1/(3*R), 1/(3*R), 1/(3*R)]
])

roboclaw.SpeedM1(ADDRESS, qpps_1)
roboclaw.SpeedM2(ADDRESS, qpps_2)
roboclaw.SpeedM1(ADDRESS, qpps_3)

while True:
    try:
        status1, QPPS_1 = roboclaw.ReadSpeedM1(0x80)  
        status2, QPPS_2 = roboclaw.ReadSpeedM2(0x80)
        status3, QPPS_3 = roboclaw.ReadSpeedM1(0x81)  


        if not (status1 and status2 and status3):
            print("Error al leer QPPS")
            continue

        V1 = (QPPS_1 / PPR) * C
        V2 = (QPPS_2 / PPR) * C
        V3 = (QPPS_3 / PPR) * C

        # Vector de velocidades de las ruedas
        V_wheels = np.array([V1, V2, V3])

        # Calcular velocidades de la base
        V_base = M_inv @ V_wheels
        Vx, Vy, omega = V_base

       
        print(f"Vx: {Vx:.3f} m/s, Vy: {Vy:.3f} m/s, ω: {omega:.3f} rad/s")

        time.sleep(0.1)  
    except KeyboardInterrupt:
        print("stop")
        roboclaw.SpeedM1(0x80, 0)
        roboclaw.SpeedM2(0x80, 0)
        roboclaw.SpeedM1(0x81, 0)
        break






    
