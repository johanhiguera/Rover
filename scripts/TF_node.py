#!/usr/bin/python

import rospy
from class_TF_lineal import TF     ##lineal
##from class_TF import TF              ##Polar

# Init of program
if __name__ == '__main__':

    rospy.init_node('TF_node', anonymous=True)
    rospy.loginfo("Creando nodo TF...")
    TF()
    rospy.spin()