#!/usr/bin/python

import math
import rospy
import math

import tf2_ros
import tf2_msgs.msg

from    geometry_msgs.msg   import TransformStamped
from    geometry_msgs.msg   import Twist
from    std_msgs.msg        import Bool
from    std_msgs.msg        import Float64MultiArray
from    sensor_msgs.msg     import NavSatFix
from    sensor_msgs.msg     import Imu
from tf.transformations     import euler_from_quaternion
from tf.transformations     import quaternion_from_euler

class TF:  

    def __init__(self):
        #CREACION DE BROADCAST
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=5)
        self.broadcts  = tf2_ros.TransformBroadcaster()
        self.transform = TransformStamped()

        #SUBSCRIPCION AL TOPICO DE COMUNICACION ENTRE TF_NODE Y CONTROLADOR_NODE
        rospy.Subscriber("/next_coord",Bool,self.callback_next)
        #rospy.Subscriber("/vel_robot",numpy_msg(Twist),self.callback_position)
        rospy.Subscriber("/rover/gps/pos",NavSatFix,self.callback_gps)
        rospy.Subscriber("/rover/imu",Imu,self.callback_imu)
        self.pub1 = rospy.Publisher("/goal",Twist,queue_size=10)

        #PARAMETROS
        self.theta = - rospy.get_param("/theta")* 3.141592 / 180.0
        self.ds = rospy.get_param("/ds")
        self.f = rospy.get_param("/f")
        self.coordenadas1 = rospy.get_param("/coordenadas")
        self.position = Float64MultiArray()
        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0
        self.first_gps = 1
        self.pos_x_init = 0
        self.pos_y_init = 0
        self.angles_odom_base = [ 0, 0, 0, 0]
        self.angles_goal_odom = [ 0, 0, 0, 0]
        self.Meta=Twist()
        self.theta_z=0

        while self.first_gps:
            self.i = 0 #VARIABLE PARA RECORRER MATRIZ SELF.TRAY (COORDENADAS DE TRAYECTORIA EN EL SISTEMA DEL ROBOT)

        self.update_coor() ##Actualizacion de coordenadas y creacion de trayectoria
        self.creacion_tray()
        #rospy.loginfo(self.coordenadas)
        rospy.loginfo(self.tray)        

        rate = rospy.Rate(self.f)

        rospy.loginfo("Nodo TF inicio correctamente ")
        
        while (not rospy.is_shutdown()):
            if(self.i < len(self.tray)-1):
                self.update_odom() ## Se envia odometria y meta en tf
                self.update_goal(self.i+1)
                self.Meta.linear.x = self.tray [self.i+1][0]
                self.Meta.linear.y = self.tray [self.i+1][1]
                self.Meta.linear.z = 0.0
                self.Meta.angular.x = 0.0
                self.Meta.angular.y = 0.0
                self.Meta.angular.z = self.tray [self.i+1][2]

                self.pub1.publish(self.Meta)

                rate.sleep()              
    
    def callback_gps(self,data): ## Callback del GPS
        if (self.first_gps == 1):
            #rospy.loginfo("Primerta posicion tomada")
            self.first_gps = 0                          ## Toma de posiciones iniciales
            self.pos_y_init = -data.longitude*111111
            self.pos_x_init = data.latitude*111111
            self.pos_y = -data.longitude*111111
            self.pos_x = data.latitude*111111 
        else:
            self.pos_y = -data.longitude*111111 ##Posicion actual x
            self.pos_x = data.latitude*111111##Posicion actual y
            #rospy.loginfo("Posicion GPS registrada")
    
    def callback_imu(self,data): #Callback imu
        self.a_x = data.linear_acceleration.x
        self.a_y = data.linear_acceleration.y
        self.a_z = data.linear_acceleration.z
        self.w_x = data.angular_velocity.x
        self.w_y = data.angular_velocity.y
        self.w_z = data.angular_velocity.y              #Paso de angulos de quartenion a euler
        thetas = euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
        self.theta_x = thetas[0] - math.pi
        self.theta_y = thetas[1]
        self.theta_z = thetas[2]    ##Agulo Yaw de nuestro robot (Respecto de coordenadas mundiales)
        #rospy.loginfo("Lectura IMU registrada")

    def update_coor(self):                      #FUNCION PARA CAMBIAR LAS COORDENADAS EN TERMINOS DEL SISTEMA DE REFERENCIA DEL ROBOT
        self.coordenadas = []
        for i in range(len(self.coordenadas1)):
            self.coordenadas.append(0)

        for i in range(len(self.coordenadas1)):
            alpha = math.atan2(self.coordenadas1[i][1],self.coordenadas1[i][0])
            r = math.sqrt(self.coordenadas1[i][1]**2+self.coordenadas1[i][0]**2)
            self.coordenadas[i] = (r*math.cos(alpha+self.theta),r*math.sin(alpha+self.theta))



    def creacion_tray(self):                    #FUNCION PARA CREAR Y LLENAR LA MATRIZ SELF.TRAY
        self.tray = []
        a=1

        for j in range(len(self.coordenadas)):
            if (j == 0): 
                dx = self.coordenadas[j+1][0]-self.coordenadas[j][0]
                dy = self.coordenadas[j+1][1]-self.coordenadas[j][1]
                s = math.sqrt(dx**2+dy**2)
                n = s/self.ds
            elif (j <= len(self.coordenadas)-1):
                dx = self.coordenadas[j][0]-self.coordenadas[j-1][0]
                dy = self.coordenadas[j][1]-self.coordenadas[j-1][1]
                s = math.sqrt(dx**2+dy**2)
                n = s/self.ds
            
            if(abs(n - int(n)) < 1E-3):
                k = int(n)
            else:
                k = int(n)+1
            
            self.tray.append(0)
            self.tray[0]=(0,0,0)

            if j == 0:
                self.tray.append(0)
            elif (j == 1) or (j == (len(self.coordenadas)-1)):    
                for i in range(k):
                    self.tray.append(0)
            elif j > 1:
                for i in range(k+1):
                    self.tray.append(0)

            if j != 0:
                for b in range(k):

                    if (j!=1):
                        self.tray[a]=(self.coordenadas[j-1][0]+(b)*dx*self.ds/s,self.coordenadas[j-1][1]+(b)*dy*self.ds/s,self.tray[a-1][2])
                        a=a+1

                    elif (b!=0):
                        self.tray[a]=(self.coordenadas[j-1][0]+(b)*dx*self.ds/s,self.coordenadas[j-1][1]+(b)*dy*self.ds/s,self.tray[a-1][2])
                        a=a+1
                    
                    if (b == 0) and (j!=1):
                        self.tray[a]=(self.tray[a-1][0],self.tray[a-1][1], math.atan2(dy,dx))  ####CALCULO DE ANGULO
                        a=a+1 
 
            if j == 0:
                self.tray[a]=(self.coordenadas[j][0],self.coordenadas[j][0], math.atan2(dy,dx))  
                a=a+1

        self.last_coor = (self.coordenadas[len(self.coordenadas)-1][0],self.coordenadas[len(self.coordenadas)-1][1],self.tray[a-1][2])
        self.tray.append(self.last_coor)
        rospy.set_param('/num_coor_tray', len(self.tray))

    def callback_next(self,data):     #FUNCION DE INTERRUPION PARA CAMBIAR A LA SIGUIENTE COORDENADA DE TRAYECTORIA
        if self.i < rospy.get_param("/num_coor_tray"):
            self.i = self.i + 1

        #rospy.loginfo("CAMBIO DE COORDENADA")

    def update_goal(self, i): #FUNCION PARA ACTUALIZAR LA MATRIZ DE TRANSFORMACION ODOM-GOAL PERIODICAMENTE

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "Goal"
        t.transform.translation.x = self.tray[i][0]
        t.transform.translation.y = self.tray[i][1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0 #self.angles_goal_odom [0]
        t.transform.rotation.y = 0.0 #self.angles_goal_odom [1]
        t.transform.rotation.z = 0.0 #self.angles_goal_odom [2]
        t.transform.rotation.w = 1.0 #self.angles_goal_odom [3]

        self.broadcts.sendTransform(t)
        rospy.loginfo("Coordenadas de meta")
        rospy.loginfo(self.tray[i][0])
        rospy.loginfo(self.tray[i][1])
        rospy.loginfo(self.tray[i][2])
        

    def update_odom(self): #FUNCION PARA ACTUALIZAR LA MATRIZ DE TRANSFORMACION ODOM-ROBOT PERIODICAMENTE
        

        to = TransformStamped()
        to.header.frame_id = "odom"
        to.header.stamp = rospy.Time.now()
        to.child_frame_id = "base_footprint"
        to.transform.translation.x = (self.pos_x-self.pos_x_init)##Posicion en x del robot
        to.transform.translation.y = (self.pos_y-self.pos_y_init)##Posicion en y del robot
        to.transform.translation.z = 0.0

        to.transform.rotation.x = 0.0 #self.angles_odom_base [0]
        to.transform.rotation.y = 0.0 #self.angles_odom_base [1]
        to.transform.rotation.z = 0.0 #self.angles_odom_base [2]
        to.transform.rotation.w = 1.0 #self.angles_odom_base [3]

        self.broadcts.sendTransform(to)   
