#!/usr/bin/env python
# -*- coding: utf8 -*-

import os
import sys
import argparse

import rospy

import numpy as np

import cv2
import PIL
import cv_bridge

import roslib
#import baxter_interface

#from baxter_interface import CHECK_VERSION

from PIL import (
    Image,
    ImageFont,
    ImageDraw
)

from sensor_msgs.msg import (
    Image as ImageMsg
)

from std_msgs.msg import String

from chaines import *
from utils import *

def main():

    POS_x = 60
    POS_y = 10
    TEXT_size = 60

    # Convert an ImageMsg to an openCV2 image
    def msg_to_cv ( imgmsg ):
        return cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, desired_encoding='bgr8')

    # Convert an openCV2 image to an ImageMsg
    def cv_to_msg ( img ):
        return cv_bridge.CvBridge().cv2_to_imgmsg( img, encoding='bgr8')

    # Publish the image of the camera to baxter screen with a text written on it
    def republish(imagemsg):

        imcv = msg_to_cv ( imagemsg )
        cv2.rectangle(imcv, (0, 0), (1024, 100 ), (255, 255, 255), -1 )
        cv2.putText(imcv, my_text, (20,70), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), thickness=3)
        
        msg = cv_to_msg ( imcv )

        """
            Sends the camera image to baxter's display
        """
        display_pub.publish(msg)
        rospy.sleep( 0.07 )


    # Met à jour le texte à afficher
    def texteCallback ( msg ):
        global my_text

        log ( msg.data )
        my_text = msg.data

    rospy.init_node('afficheur')      

    # Lis les messages sur le flux rob4/out et les affiche à l'écran
    out_sub = rospy.Subscriber ( TOPIC_OUT, String, texteCallback, None, 1 )

    #    Initialisation
    texteCallback ( String ( "booting up.." ) )
    rospy.sleep( 1 )

    cam_sub = rospy.Subscriber( "image_raw", ImageMsg, republish, None, 1)
    display_pub = rospy.Publisher('/robot/xdisplay', ImageMsg, queue_size=1) 
    
    rospy.spin()

if __name__ == '__main__':
    main()
