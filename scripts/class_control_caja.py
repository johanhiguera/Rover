#!/usr/bin/python

import rospy
from   std_msgs.msg         import Float64
from   sensor_msgs.msg      import JointState

class CONTROL_CAJA:  

    def __init__(self):
        self.nameTopicPub1 = "/rocker_der/command"
        self.nameTopicSub1 = "/joint_states"

        self.pub1 = rospy.Publisher(self.nameTopicPub1,Float64,queue_size=10)
        rospy.Subscriber(self.nameTopicSub1,JointState,self.callback)
        
        rate = rospy.Rate(10)
        self.position_caja = 0

        while (not rospy.is_shutdown()):
            self.nivelar_caja()
            rate.sleep()
    
    def callback(self,data):
        if("Caja__rocker_izq" in data.name):
            index = data.name.index("Caja__rocker_izq")
            self.position_caja = -data.position[index]
            
    def nivelar_caja(self):
        self.pub1.publish(self.position_caja)