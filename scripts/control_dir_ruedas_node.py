#!/usr/bin/python

import rospy
from class_control_dir_ruedas import CONTROL_DIR_RUEDAS

# Init of program
if __name__ == '__main__':

    rospy.init_node('Nodo de control de direccion de ruedas', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    CONTROL_DIR_RUEDAS()

    rospy.spin()