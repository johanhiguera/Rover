#!/usr/bin/python

import rospy
from class_seguimiento_visual import VISUAL     ##lineal
##from class_TF import TF              ##Polar

# Init of program
if __name__ == '__main__':

    rospy.init_node('Seguimiento_visual_node', anonymous=True)
    rospy.loginfo("Creando nodo de seguimiento visual...")
    VISUAL()
    rospy.spin()