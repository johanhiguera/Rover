#!/usr/bin/python

import rospy
import math

from   std_msgs.msg         import Float64
from   geometry_msgs.msg    import Twist
from   std_msgs.msg         import Float64MultiArray

class CONTROL_DIR_RUEDAS:  

    def __init__(self):
        self.vel2send = Twist()
        self.velWheel2send = Float64MultiArray()

        self.dir_order = [0,0,0,0]
        self.vel_y = 0
        self.w = 0

        self.nameTopicSub1 = "/vel_order"

        self.nameTopicPub1 = "/dir_right_llanta_1/command"
        self.nameTopicPub2 = "/dir_right_llanta_3/command"
        self.nameTopicPub3 = "/dir_left_llanta_1/command"
        self.nameTopicPub4 = "/dir_left_llanta_3/command"

        rospy.Subscriber(self.nameTopicSub1,Float64MultiArray,self.callback)

        self.pub1 = rospy.Publisher(self.nameTopicPub1,Float64,queue_size=10)
        self.pub2 = rospy.Publisher(self.nameTopicPub2,Float64,queue_size=10)
        self.pub3 = rospy.Publisher(self.nameTopicPub3,Float64,queue_size=10)
        self.pub4 = rospy.Publisher(self.nameTopicPub4,Float64,queue_size=10)

        # Para ruedas derechas
        self.alpha_r1 = rospy.get_param("/alpha_r1")
        self.l_r1     = rospy.get_param("/l_r1")
        self.alpha_r2 = rospy.get_param("/alpha_r2")
        self.l_r2     = rospy.get_param("/l_r2")
        self.alpha_r3 = rospy.get_param("/alpha_r3")
        self.l_r3     = rospy.get_param("/l_r3")

        # Para ruedas izquierdas
        self.alpha_l1 = rospy.get_param("/alpha_l1")
        self.l_l1     = rospy.get_param("/l_l1")
        self.alpha_l2 = rospy.get_param("/alpha_l2")
        self.l_l2     = rospy.get_param("/l_l2")
        self.alpha_l3 = rospy.get_param("/alpha_l3")
        self.l_l3     = rospy.get_param("/l_l3")

        rate = rospy.Rate(10)
        self.vel_y = 0
        self.w = 0
        self.quit = False
        self.key = ' '


        while (not rospy.is_shutdown()):
            
            self.calcular_angulos()
            self.controlar_direccion()
            rate.sleep()
    
    def callback(self,order):
        self.vel_y = order.data[0]
        self.w = order.data[1]
    
    def calcular_angulos(self):
        if math.fabs(self.w) > 1E-5:
            R = self.vel_y / self.w
            self.dir_order[0] = math.pi - math.atan2(self.l_r1*math.sin(self.alpha_r1),R-self.l_r1*math.cos(self.alpha_r1))
            self.dir_order[1] = math.pi - math.atan2(self.l_r3*math.sin(self.alpha_r3),R-self.l_r3*math.cos(self.alpha_r3))
            self.dir_order[2] = math.pi - math.atan2(self.l_l1*math.sin(self.alpha_l1),R-self.l_l1*math.cos(self.alpha_l1))
            self.dir_order[3] = math.pi - math.atan2(self.l_l3*math.sin(self.alpha_l3),R-self.l_l3*math.cos(self.alpha_l3))
            ##rospy.loginfo(self.dir_order)
        else:
            self.dir_order = [0,0,0,0]
        ##rospy.loginfo(self.dir_order)
    
    def controlar_direccion(self):
        self.pub1.publish(self.dir_order[0])
        self.pub2.publish(self.dir_order[1])
        self.pub3.publish(self.dir_order[2])
        self.pub4.publish(self.dir_order[3])
