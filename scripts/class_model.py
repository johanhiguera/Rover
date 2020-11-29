#!/usr/bin/python

import rospy
import numpy as np

from   sensor_msgs.msg      import JointState

class Model_robot:
    def __init__(self):
        self.nameTopicSub1 = "/joint_states"
        rospy.Subscriber(self.nameTopicSub1,JointState,self.callback)
        self.dir_llantas = [0,0,0,0]

        self.actualizar_modelo()
        

    def actualizar_modelo(self):
        # Parametros de las ruedas
        radius = rospy.get_param("/radius")

        # Para ruedas derechas
        alpha_r1 = rospy.get_param("/alpha_r1")
        beta_r1  = rospy.get_param("/beta_r1") + self.dir_llantas[0]
        l_r1     = rospy.get_param("/l_r1")
        
        alpha_r2 = rospy.get_param("/alpha_r2")
        beta_r2  = rospy.get_param("/beta_r2")
        l_r2     = rospy.get_param("/l_r2")

        alpha_r3 = rospy.get_param("/alpha_r3")
        beta_r3  = rospy.get_param("/beta_r3") + self.dir_llantas[1]
        l_r3     = rospy.get_param("/l_r3")

        # Para ruedas izquierdas
        alpha_l1 = rospy.get_param("/alpha_l1")
        beta_l1  = rospy.get_param("/beta_l1") + self.dir_llantas[2]
        l_l1     = rospy.get_param("/l_l1")

        alpha_l2 = rospy.get_param("/alpha_l2")
        beta_l2  = rospy.get_param("/beta_l2")
        l_l2     = rospy.get_param("/l_l2")

        alpha_l3 = rospy.get_param("/alpha_l3")
        beta_l3  = rospy.get_param("/beta_l3") + self.dir_llantas[3]
        l_l3     = rospy.get_param("/l_l3")

        # Definicion matrices J1 y J2
        J1 = np.array(  [ (np.sin(alpha_r1+beta_r1),  -np.cos(alpha_r1+beta_r1), -l_r1*np.cos(beta_r1) ),
                          (np.sin(alpha_r2+beta_r2),  -np.cos(alpha_r2+beta_r2), -l_r2*np.cos(beta_r2) ),
                          (np.sin(alpha_r3+beta_r3),  -np.cos(alpha_r3+beta_r3), -l_r3*np.cos(beta_r3) ),
                          (np.sin(alpha_l1+beta_l1),  -np.cos(alpha_l1+beta_l1), -l_l1*np.cos(beta_l1) ),
                          (np.sin(alpha_l2+beta_l2),  -np.cos(alpha_l2+beta_l2), -l_l2*np.cos(beta_l2) ),
                          (np.sin(alpha_l3+beta_l3),  -np.cos(alpha_l3+beta_l3), -l_l3*np.cos(beta_l3) )])

        J2 = np.array([(radius , 0      , 0      , 0     , 0      , 0     ),
                       (0      , radius , 0      , 0     , 0      , 0     ),
                       (0      , 0      , radius , 0     , 0      , 0     ),
                       (0      , 0      , 0      ,radius , 0      , 0     ),
                       (0      , 0      , 0      , 0     , radius , 0     ),
                       (0      , 0      , 0      , 0     , 0      , radius)])

        # Definir Jacob_inv

        self.Jacob_inv1 = np.matmul ( np.linalg.pinv(J2) , J1 )
        self.Jacob_inv2 = np.matmul ( np.linalg.pinv(J1) , J2 )

    def callback(self,data):
        if("Bogie_der__acople_4" in data.name):
            index = data.name.index("Bogie_der__acople_4")
            self.dir_llantas[0] = -data.position[index]
        
        if("Rocker_der__acople_6" in data.name):
            index = data.name.index("Rocker_der__acople_6")
            self.dir_llantas[1] = -data.position[index]
        
        if("Bogie_izq__acople_1" in data.name):
            index = data.name.index("Bogie_izq__acople_1")
            self.dir_llantas[2] = -data.position[index]
        
        if("Rocker_izq__acople_3" in data.name):
            index = data.name.index("Rocker_izq__acople_3")
            self.dir_llantas[3] = -data.position[index]


    def calcVelWheels (self, vel_y, vel_ang):
        vec_Vel = np.array([(0),(vel_y),(-vel_ang)]) # Velocidad en el marco del robot
        velWheels = np.matmul(self.Jacob_inv1, vec_Vel)
        return velWheels

    def calcVel (self, vel_r1, vel_r2, vel_r3, vel_l1, vel_l2, vel_l3):
        vec_Wheels = np.array([[vel_r1],[vel_r2],[vel_r3],[vel_l1],[vel_l2],[vel_l3]])
        vec_Vel = np.matmul(self.Jacob_inv2,vec_Wheels)
        return vec_Vel