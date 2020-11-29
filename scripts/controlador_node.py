#!/usr/bin/python

import rospy
from class_control_lineal import CONTROL

# Init of program
if __name__ == '__main__':

    rospy.init_node('CNTL_node', anonymous=True)
    rospy.loginfo("Creando nodo controlador...")
    CONTROL()  
    rospy.spin()