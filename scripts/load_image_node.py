#!/usr/bin/env python

import rospy

from rospy.numpy_msg import numpy_msg
import numpy as np
import numpy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from    std_msgs.msg        import Float64MultiArray

import sys
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Image

class Image_loader:

    #------------------------------------------------------#
    # Callback function for image
    def cb_image(self, data):
        self.image_width = data.width
        self.image_height = data.height
        self.image_length = data.step

        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

        self.image_pub = rospy.Publisher("image_objects", Image, queue_size=1)

    #------------------------------------------------------#

    def __init__(self):

        self.image_width = None
        self.image_height = None
        self.image_length = None
        self.radii = 0
        self.x_cor = 0
        self.y_cor = 0
        self.datos_imagen = Float64MultiArray()


        self.p=0
        self.send_datos_edited_image = Image()

        # Create CV bridge
        self.bridge = CvBridge()

        # Create image subscriber
        rospy.Subscriber("/rover/camera/image_raw", numpy_msg(Image), self.cb_image)
        
        ##Publicador de la nueva imagen
        self.pub_image = rospy.Publisher("/edited_image",numpy_msg(Image), queue_size = 10)
        self.pub1 = rospy.Publisher("/pelota",Float64MultiArray,queue_size=50)

        self.image_pub = None

        self.spin()

    #------------------------------------------------------#
    # Spin function
    def spin(self):

        rate = rospy.Rate(30)


        while (not rospy.is_shutdown()):

            if(self.image_pub is not None):

                cv_image = np.zeros((self.image_width,self.image_height), dtype = np.uint8)
                cv_image = self.image.copy()

                imagenHSV = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
                amarilloBajo = np.array([20, 100, 20], np.uint8)
                amarilloAlto = np.array([32, 255, 255], np.uint8)
                maskAmarillo = cv2.inRange(imagenHSV, amarilloBajo, amarilloAlto)

                #gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
                #circles1 = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1.31,260,param1=50,param2=30,minRadius=0,maxRadius=0)
                
                circles2 = cv2.HoughCircles(maskAmarillo,cv2.HOUGH_GRADIENT,1.31,260,param1=50,param2=30,minRadius=0,maxRadius=0)

                if circles2 is not None:
                    
                    circles2 = np.round(circles2[0, :]).astype("int")
                    
                    for (x, y, r) in circles2:
                        self.radii = r
                        self.x_cor = x
                        self.y_cor = y
                        cv2.circle(cv_image, (x, y), r, (0, 255, 255), 2)
                        cv2.rectangle(cv_image, (x - 2, y - 2), (x + 2, y + 2), (0, 128, 255), -1)
                
                #rospy.loginfo(self.radii)
                #radii = circles[:,2]
                #x_cor = circles[:,0]
                #y_cor = circles[:,1]

                xdis = self.x_cor - int(self.image_width / 2)
                ydis = self.y_cor - int(self.image_width / 2)

                self.datos_imagen.data = [xdis,ydis,self.radii]

                self.edited_image = cv_image

                self.send_datos_edited_image = self.bridge.cv2_to_imgmsg(self.edited_image, encoding="rgb8")

                if self.p==0:
                    #rospy.loginfo("---------------------------------")
                    #rospy.loginfo(self.send_datos_edited_image)
                    self.p=1
                self.pub_image.publish(self.send_datos_edited_image)
                self.pub1.publish(self.datos_imagen)
                 
            rate.sleep()


        print("Shutting down")
        cv2.destroyAllWindows()

#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    # Firt init the node and then the object to correctly find the parameters
    rospy.init_node('image_loader', anonymous=True)
    Image_loader()






    