#!/usr/bin/python

import rospy

from    std_msgs.msg        import Float64MultiArray

class CONTROL:  

    def __init__(self):
        #PARAMETROS
        self.f = rospy.get_param("/f")
        self.vel_cruc = rospy.get_param("/vel_cruc")
        self.w_max = rospy.get_param("/w_max")
        self.Kp = rospy.get_param("/Kp_1")

        #VARIABLES PARA GUARDAR LA INFORMACION DE LA PELOTA SEGUN LA CAMARA
        self.pos_x=0.0
        self.pos_y=0.0
        self.radius=10000
        self.radius_ref = 50

        rospy.Subscriber("/pelota",Float64MultiArray,self.callback_pelota)

        #COMUNICACION ENTRE EL NODO DE SEGUIMIENTO Y KOBUKI
        self.pub1 = rospy.Publisher("/vel_order",Float64MultiArray,queue_size=50)

        #MENSAJES PARA PUBLICACION EN TOPICOS
        self.order = Float64MultiArray()
        self.vel_y = 0.0
        self.w = 0.0
        
        rate = rospy.Rate(self.f)

        while(self.radius==10000):
            rate.sleep()

        rospy.loginfo("Nodo seguimiento visual inicio correctamente ")
        
        while (not rospy.is_shutdown()):
            self.Controlador_polar()
            rate.sleep()

    def callback_pelota(self,data):
        self.pos_x = data[0]
        self.pos_y = data[1]
        self.radius = data[2]

    def Controlador_polar(self):                #FUNCION PARA EL CONTROL EN SISTEMA POLAR DEL ROVER
        self.vel_y = self.Kp[0]*(self.radius-self.radius_ref)
        self.w = self.Kp[1]*self.pos_x

        if self.vel_y > self.vel_cruc:
            self.vel_y = self.vel_cruc
        elif self.vel_y < -self.vel_cruc:
            self.vel_y = -self.vel_cruc

        if self.w > self.w_max:
            self.w = self.w_max
        elif self.w < -self.w_max:
            self.w = -self.w_max

        self.order.data = [self.vel_y,self.w]
        self.pub1.publish(self.order)