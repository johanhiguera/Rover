#!/usr/bin/python

import rospy
import sys, select, termios, tty
from   std_msgs.msg         import Float64MultiArray

class LECTURE_KEY:  

    def __init__(self):
        self.order = Float64MultiArray()

        self.v_crucero = rospy.get_param("/vel_cruc")
        self.w_max = rospy.get_param("/w_max")
        self.f = rospy.get_param("/f")

        self.key_timeout = rospy.get_param("~key_timeout", 0.0)
        if self.key_timeout == 0.0:
                self.key_timeout = None

        self.nameTopicPub1 = "/vel_order"

        self.pub1 = rospy.Publisher(self.nameTopicPub1,Float64MultiArray,queue_size=10)
        
        rate = rospy.Rate(self.f)
        self.vel_y = 0
        self.w = 0
        self.quit = False
        self.key = ' '

        while (not rospy.is_shutdown()):
            if(~self.quit):
                self.detectar_key()
            else:
                break
            
            rate.sleep()
    
    def detectar_key(self):
        
        self.key = getKey(self,self.key_timeout)
        
        if self.key in moveBindings.keys():
            if ((self.key == 'q') | (self.key == 'Q')):
                self.quit = True
            elif (self.key == ' '):
                self.vel_y = moveBindings[self.key][0]
                self.w = moveBindings[self.key][1]
            else:
                self.vel_y = self.vel_y + moveBindings[self.key][0]
                self.w = self.w + moveBindings[self.key][1]
            
            if (self.vel_y >= self.v_crucero):
                self.vel_y = self.v_crucero
            elif (self.vel_y <= -self.v_crucero):
                self.vel_y = -self.v_crucero
            
            if (self.w >= self.w_max):
                self.w = self.w_max
            elif (self.w <= -self.w_max):
                self.w = -self.w_max

            self.order.data = [self.vel_y,self.w]
            self.pub1.publish(self.order)

settings = termios.tcgetattr(sys.stdin)

def getKey(self,key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

moveBindings = {
        's':(-0.1,0.0),
        'S':(-0.1,0.0),
        'w':(0.1,0.0),
        'W':(0.1,0.0),
        'a':(0.0,-0.3),
        'A':(0.0,-0.3),
        'd':(0.0,0.3),
        'D':(0.0,0.3),
        ' ':(0.0,0.0),
        'q':(0.0,0.0),
        'Q':(0.0,0.0)
    }