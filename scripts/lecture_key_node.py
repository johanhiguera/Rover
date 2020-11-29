#!/usr/bin/python

import rospy
from class_lecture_key import LECTURE_KEY

# Init of program
if __name__ == '__main__':

    rospy.init_node('key_press', anonymous=True)

    rospy.loginfo("RyCSV__2020-2")

    LECTURE_KEY()

    rospy.spin()