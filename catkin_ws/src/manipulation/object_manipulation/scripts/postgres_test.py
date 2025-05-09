#!/usr/bin/env python3

import rospy 
import psycopg2
import ros_numpy
import numpy as np
import math
import os

# Establecer los parametros de conexion
dbname   = 'grasp_database'
user     = 'postgres'
password = 'pumas'
host     = 'localhost'
port     = '5432'

conn = None
# Conectar a la bd
try:
    conn = psycopg2.connect(dbname=dbname, user=user,password=password, host=host, port=port)
    print("Conexión de base de datos exitosa")
    #crea el cursor
    cur = conn.cursor()
    #Consulta a la base de datos
    cur.execute("SELECT *FROM test")
    #Obtiene el resultado
    rows = cur.fetchall()
    for row in rows:
        print(row)

    # Cierra el cursor
    cur.close()

except psycopg2.Error as error:
    print("Error al establecer la conexion:", error)
finally:
    if conn is not None:
        conn.close()
        print("Conexion cerrada")