#!/usr/bin/python

import rospy

from    std_msgs.msg        import Float64MultiArray

class VISUAL:  

    def __init__(self):
        #PARAMETROS
        self.f = rospy.get_param("/f")
        self.vel_cruc = rospy.get_param("/vel_cruc")
        self.w_max = rospy.get_param("/w_max")
        self.Kp = rospy.get_param("/Kp_1")
        self.Ki = rospy.get_param("/Ki")

        #VARIABLES PARA GUARDAR LA INFORMACION DE LA PELOTA SEGUN LA CAMARA
        self.pos_x=0.0
        self.sum_pos_x=0.0
        self.pos_y=0.0
        self.radius=10000
        self.radius_ref = 17
        self.rot = False

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
        self.pos_x = data.data[0]
        self.pos_y = data.data[1]
        self.radius = data.data[2]
        #rospy.loginfo(self.radius)

    def Controlador_polar(self):                #FUNCION PARA EL CONTROL EN SISTEMA POLAR DEL ROVER      
        if(abs(self.pos_x)>40 or self.rot):
            self.sum_pos_x = self.sum_pos_x + self.pos_x/self.f
            self.vel_y = 0
            self.w = self.Kp[1]*self.pos_x #+ self.Ki*self.sum_pos_x

            if self.w > self.w_max:
                self.w = self.w_max
            elif self.w < -self.w_max:
                self.w = -self.w_max
            
            if(abs(self.pos_x)<5):
                self.rot = False
        else:       
            self.vel_y = self.Kp[0]*(self.radius_ref-self.radius)
            self.w = 0

            if self.vel_y > self.vel_cruc:
                self.vel_y = self.vel_cruc
            elif self.vel_y < -self.vel_cruc:
                self.vel_y = -self.vel_cruc

        self.order.data = [self.vel_y,self.w]
        self.pub1.publish(self.order)