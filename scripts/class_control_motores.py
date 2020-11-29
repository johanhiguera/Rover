#!/usr/bin/python

import rospy

from   std_msgs.msg         import Float64
from   geometry_msgs.msg    import Twist
from   rospy.numpy_msg      import numpy_msg
from   std_msgs.msg         import Float64MultiArray

from   class_model          import Model_robot

class CONTROL_MOTORES:  

    def __init__(self):
        self.vel2send = Twist()
        self.velWheel2send = Float64MultiArray()

        self.vel_y = 0
        self.w = 0

        self.modelo = Model_robot()

        self.nameTopicSub1 = "/vel_order"

        self.nameTopicPub1 = "/right_motor_1/command"
        self.nameTopicPub2 = "/right_motor_2/command"
        self.nameTopicPub3 = "/right_motor_3/command"
        self.nameTopicPub4 = "/left_motor_1/command"
        self.nameTopicPub5 = "/left_motor_2/command"
        self.nameTopicPub6 = "/left_motor_3/command"
        self.nameTopicPub7 = "/vel_robot"
        self.nameTopicPub8 = "/vel_wheel"

        rospy.Subscriber(self.nameTopicSub1,Float64MultiArray,self.callback)

        self.pub1 = rospy.Publisher(self.nameTopicPub1,Float64,queue_size=10)
        self.pub2 = rospy.Publisher(self.nameTopicPub2,Float64,queue_size=10)
        self.pub3 = rospy.Publisher(self.nameTopicPub3,Float64,queue_size=10)
        self.pub4 = rospy.Publisher(self.nameTopicPub4,Float64,queue_size=10)
        self.pub5 = rospy.Publisher(self.nameTopicPub5,Float64,queue_size=10)
        self.pub6 = rospy.Publisher(self.nameTopicPub6,Float64,queue_size=10)
        self.pub7 = rospy.Publisher(self.nameTopicPub7,numpy_msg(Twist),queue_size=10)
        self.pub8 = rospy.Publisher(self.nameTopicPub8,Float64MultiArray,queue_size=10)

        
        rate = rospy.Rate(10)
        self.vel_y = 0
        self.w = 0
        self.quit = False
        self.key = ' '


        while (not rospy.is_shutdown()):
            self.modelo.actualizar_modelo()
            self.control_ruedas()
            self.enviar_odometria()
            rate.sleep()
    
    def callback(self,order):
        self.vel_y = order.data[0]
        self.w = order.data[1]

    def control_ruedas(self):
        self.velWheel = self.modelo.calcVelWheels(self.vel_y, self.w)
        self.velWheel2send.data = [self.velWheel[0],self.velWheel[1],self.velWheel[2],self.velWheel[3],self.velWheel[4],self.velWheel[5]]
        self.pub1.publish(self.velWheel[0])
        self.pub2.publish(self.velWheel[1])
        self.pub3.publish(self.velWheel[2])
        self.pub4.publish(-self.velWheel[3])
        self.pub5.publish(-self.velWheel[4])
        self.pub6.publish(-self.velWheel[5])
        self.pub8.publish(self.velWheel2send)

    def enviar_odometria(self):
        self.vel = self.modelo.calcVel(self.velWheel[0],self.velWheel[1],self.velWheel[2],self.velWheel[3],self.velWheel[4],self.velWheel[5])
        self.vel2send.linear.x = self.vel[0]
        self.vel2send.linear.y = self.vel[1]
        self.vel2send.angular.z = self.vel[2]
        self.pub7.publish(self.vel2send)
