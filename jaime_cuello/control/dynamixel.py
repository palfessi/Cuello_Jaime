#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import sys
import rospy
from dynamixel_workbench_msgs.srv import *
from std_msgs.msg import Float64MultiArray

class IMU():
    def __init__(self):
        self.IMU=list()
        self.suscriber=rospy.Subscriber("gyroscope", Float64MultiArray, self.callback)
        
    def callback(self, data):

        rospy.loginfo(data.data)
        x, y ,z= data.data[0], data.data[1],data.data[2]
        print("Received x={}, y={}".format(x, y))

        # Llama a la función goal_position para mover el servo

        #tercer motor
        goal_position(3, int(x), min=0, max=341)
        #Cuarto motor
        goal_position(4, int(y), min=240, max=855)
        #Quinto motor
        goal_position(5, int(z), min=230, max=820)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('Dynamixel ', anonymous=True)

        rospy.Subscriber("gyroscope", Float64MultiArray, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def goal_position(ID , pos , min=0,max=1023 ):
    #rospy.wait_for_service('/dynamixel_command')

    #Se delimita el movimiento
    if pos<min:
        pos=min

    if pos>max:
        pos=max


    try:
        #Se manda el servicio para mover el motor
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command' , DynamixelCommand)
        print ("ServiceProxy success ...")
            
        resp= dynamixel_command( '',ID,'Goal_Position',pos)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    # Inicializa la clase IMU
    imu = IMU()
    
    # Llama a la función listener para iniciar la suscripción al tópico "gyroscope"
    imu.listener()

        
        