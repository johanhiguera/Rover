#!/usr/bin/python

import rospy
from class_control_motores import CONTROL_MOTORES

# Init of program
if __name__ == '__main__':

    rospy.init_node('control_motores', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    CONTROL_MOTORES()

    rospy.spin()