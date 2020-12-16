#!/usr/bin/env python

import rospy

from rospy.numpy_msg import numpy_msg
import numpy as np
import numpy

import cv2
from cv_bridge import CvBridge, CvBridgeError

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

        self.p=0
        self.send_datos_edited_image = Image()

        # Create CV bridge
        self.bridge = CvBridge()

        # Create image subscriber
        rospy.Subscriber("/rover/camera/image_raw", numpy_msg(Image), self.cb_image)
        
        ##Publicador de la nueva imagen
        self.pub_image = rospy.Publisher("/edited_image",numpy_msg(Image), queue_size = 10)

        self.image_pub = None

        self.spin()

    #------------------------------------------------------#
    # Spin function
    def spin(self):

        rate = rospy.Rate(30)


        while (not rospy.is_shutdown()):

            if(self.image_pub is not None):

                cv_image = np.zeros((self.image_width,self.image_height), dtype = np.uint8)
                base_image = np.zeros((self.image_width,self.image_height), dtype = np.uint8)
                cv_image = self.image.copy() ##Se recibe la imagen del mapa
                base_image = self.image.copy()              

                ###Aqui va el codigo de Sebastian

                self.edited_image = cv_image

                self.send_datos_edited_image = self.bridge.cv2_to_imgmsg(self.edited_image, encoding="rgb8")

                if self.p==0:
                    #rospy.loginfo("---------------------------------")
                    #rospy.loginfo(self.send_datos_edited_image)
                    self.p=1
                self.pub_image.publish(self.send_datos_edited_image)
                 
            rate.sleep()


        print("Shutting down")
        cv2.destroyAllWindows()

#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    # Firt init the node and then the object to correctly find the parameters
    rospy.init_node('image_loader', anonymous=True)
    Image_loader()
    