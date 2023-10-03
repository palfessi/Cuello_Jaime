#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
from std_msgs.msg import Float64MultiArray
import time 

# Configura la conexión serie con Arduino
arduino_port = '/dev/ttyUSB0'  # Cambia esto al puerto correcto
baud_rate = 115200  # Asegúrate de que coincide con la velocidad de transmisión de Arduino

# Inicializa el nodo ROS
rospy.init_node('gyro_publisher', anonymous=True)

# Crea un objeto para publicar los datos
pub = rospy.Publisher('gyro', Float64MultiArray, queue_size=10)

try:
    arduino = serial.Serial(arduino_port, baud_rate)

    while not rospy.is_shutdown():
        # Lee una línea de datos desde Arduino
        line = arduino.readline().decode('utf-8').strip()
        valor_arduino = str(line)
        grados = valor_arduino.split(",")
        
        # Convierte los valores a tipo float
        try:
            grados = [float(grado) for grado in grados]
        except ValueError:
            rospy.logwarn("No se pudo convertir a float: {}".format(grados))
            continue

        # Crea un mensaje Float64MultiArray y publícalo
        data = Float64MultiArray()
        data.data = grados
        pub.publish(data)
        
        

except serial.SerialException:
    rospy.logerr("No se pudo conectar a Arduino en {}".format(arduino_port))
finally:
    arduino.close()