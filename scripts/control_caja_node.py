#!/usr/bin/python

import rospy
from class_control_caja import CONTROL_CAJA

# Init of program
if __name__ == '__main__':

    rospy.init_node('control_caja', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    CONTROL_CAJA()

    rospy.spin()