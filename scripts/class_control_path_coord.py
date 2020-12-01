#!/usr/bin/python

import math
import rospy
import numpy as np

import tf2_ros
import tf2_msgs.msg
import tf_conversions

from    std_msgs.msg        import Bool
from    std_msgs.msg        import Float64MultiArray
from    sensor_msgs.msg     import Imu
from tf.transformations     import euler_from_quaternion

class CONTROL:  

    def __init__(self):
        #PARAMETROS
        self.f = rospy.get_param("/f")
        self.vel_cruc = rospy.get_param("/vel_cruc")
        self.w_max = rospy.get_param("/w_max")
        self.Kp = rospy.get_param("/Kp")
        self.dv = rospy.get_param("/dv")
        self.dw = rospy.get_param("/dw")
        self.num_coor_tray = 0     #NUMERO DE COORDENADAS DE LA TRAYECTORIA QUE SE CALCULA EN TF_NODE
        self.pos_x=0.0
        self.pos_y=0.0
        self.pos_w=0.0
        self.theta_z = 0.0
        #CREACION DEL LISTENER
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.Subscriber("/rover/imu",Imu,self.callback_imu)

        #COMUNICACION ENTRE EL CONTROLADOR_NODE Y KOBUKI
        self.pub1 = rospy.Publisher("/vel_order",Float64MultiArray,queue_size=50)

        #CREACION DEL NODO DE COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        self.pub_next_coord = rospy.Publisher("/next_coord", Bool, queue_size=1)

        #MENSAJES PARA PUBLICACION EN TOPICOS
        self.next = Bool()                      #VARIABLE PARA LA COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        self.next.data = True
        self.order = Float64MultiArray()
        self.position = Float64MultiArray()
        self.vel_y = 0.0
        self.w = 0.0
        self.theta_parcial=0
        rate = rospy.Rate(self.f)

        self.j=0                                #VARIABLE QUE INDICA CUANTOS CAMBIOS DE COORDENADAS SE HAN REALIZADO
        self.MTH = [0]                          #MATRIZ DE TRANSFORMACION ODOM-GOAL
        self.orden_anterior = [0,0]             ##Arreglo para minimizar el cambio de velocidades

        while(self.num_coor_tray == 0):
            self.num_coor_tray = rospy.get_param("/num_coor_tray")

        rospy.loginfo("Nodo controlador inicio correctamente ")
        
        while (not rospy.is_shutdown()):
            if(self.j < self.num_coor_tray):
                self.j = self.j + 1
                MTH = [0]
                while(len(self.MTH) == 1):
                    self.MTH = self.Calcular_MTH()
                    rate.sleep()

                self.rho    = 10
                self.theta  = 0
                while (self.rho >= self.vel_cruc/self.Kp[0]):
                    self.MTH = self.Calcular_MTH()
                    if(len(self.MTH) != 1):
                        self.Controlador_polar()
                    
                    rate.sleep()
                self.pub_next_coord.publish(self.next)
                rospy.loginfo ("Cambio de coordenadas")
                rate.sleep()

    
    def Calcular_MTH(self):                     #FUNCION PARA CALCULAR LA MATRIZ DE TRANSFORMACION ENTRE ROVER-GOAL
        try:
            trans_base_marker = self.tfBuffer.lookup_transform("base_footprint", "Goal", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Error trying to look for transform")
            return [0]
        quat_from_ROS = np.array([trans_base_marker.transform.rotation.x, \
                                    trans_base_marker.transform.rotation.y, \
                                    trans_base_marker.transform.rotation.z, \
                                    trans_base_marker.transform.rotation.w])
        rt_mat_from_ROS = tf_conversions.transformations.quaternion_matrix(quat_from_ROS)
        MTH_GOAL = rt_mat_from_ROS.copy()
        MTH_GOAL[0,3] = trans_base_marker.transform.translation.x
        MTH_GOAL[1,3] = trans_base_marker.transform.translation.y
        
        return MTH_GOAL

    def callback_imu(self,data): #Callback imu
        thetas = euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        self.theta_x = thetas[0] - math.pi
        self.theta_y = thetas[1]
        self.theta_z = thetas[2] + math.pi    ##Agulo Yaw de nuestro robot (Respecto de coordenadas mundiales)    
    
    def Controlador_polar(self):                #FUNCION PARA EL CONTROL EN SISTEMA POLAR DEL ROVER
        self.dx     =  self.MTH[0,3]
        self.dy     =  self.MTH[1,3]
        self.theta  =  self.theta_z
        self.rho    =  math.sqrt(self.dx**2+self.dy**2)
        self.beta   = -math.atan2(self.dy,self.dx)
        self.alpha  = -self.theta - self.beta

        self.vel_y = self.Kp[0]*self.rho
        self.w = self.Kp[1]*self.alpha + self.Kp[2]*self.beta

        if self.vel_y > self.vel_cruc:
            self.vel_y = self.vel_cruc
        elif self.vel_y < -self.vel_cruc:
            self.vel_y = -self.vel_cruc

        if self.w > self.w_max:
            self.w = self.w_max
        elif self.w < -self.w_max:
            self.w = -self.w_max

        if self.vel_y - self.orden_anterior[0] > self.dv/self.f: ##Limitacion de maximo cambio
            self.vel_y = self.orden_anterior[0] + self.dv/self.f ##en velocidad lineal
        elif self.vel_y - self.orden_anterior[0] < -self.dv/self.f:
            self.vel_y = self.orden_anterior[0] - self.dv/self.f

        if self.w - self.orden_anterior[1] > self.dw/self.f: ##Limitacion de maximo cambio
            self.w = self.orden_anterior[1] + self.dw/self.f##En velocidad angular (Giro ruedas)
        elif self.w - self.orden_anterior[1] < -self.dw/self.f:
            self.w = self.orden_anterior[1] - self.dw/self.f

        self.order.data = [self.vel_y,self.w]
        self.orden_anterior = self.order.data
        self.pub1.publish(self.order)

        rospy.loginfo("--------------------------")
        rospy.loginfo(self.dx)
        rospy.loginfo(self.dy)
        rospy.loginfo(self.theta)
        rospy.loginfo(self.rho)
        rospy.loginfo(self.beta)
        rospy.loginfo(self.alpha)
        
        #rospy.loginfo(self.vel_y)
        #rospy.loginfo(self.w)
        #rospy.loginfo("self.MTH")
        #rospy.loginfo(self.MTH)

    


