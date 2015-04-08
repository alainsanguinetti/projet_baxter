#!/usr/bin/env python
# -*- coding: utf8 -*-

import os
import sys
import argparse

import rospy

import numpy as np

import cv
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

my_text = " abc "

def main():

    POS_x = 60
    POS_y = 10
    TEXT_size = 60

    ## http://sdk.rethinkrobotics.com/wiki/Demo_Mode_-_Code_Walkthrough 
    def gen_cv(img):
        return PIL_to_cv(rgb_to_bgr(img))
     
    def rgb_to_bgr(img):
        r,g,b = img.split()
        return PIL.Image.merge('RGB', (b, g, r))
     
    def PIL_to_cv(img):
        ci = cv.CreateImage((1024, 100), cv.IPL_DEPTH_8U, 3)
        cv.SetData(ci, img.tostring(), 3072)
        return ci

    def cv_to_msg(img):
        return cv_bridge.CvBridge().cv_to_imgmsg(img, encoding='bgr8')

    def cv2_to_msg ( img ):
        return cv_bridge.CvBridge().cv2_to_imgmsg( img, encoding='bgr8')

    def msg_to_cv(img):
        return cv_bridge.CvBridge().imgmsg_to_cv2(img, desired_encoding='bgr8')

    def overlay(old_img, sub, x_overlay_offset=0, y_overlay_offset=0):
        """
        # Overlays the ROS message new_img on top of the opencv2 old_img
        # using the following coordinate system to map the new_img's
        # origin (0,0) to an (x,y) position on the old image.
        # The coordinate frame for both images the opencv2 standard:
        # 0-->
        # |  x
        # v y
        """
        # Copy out the original image
        tmp = np.copy(old_img)

        # Guard against running over the bounds of temp image
        y_overlay_end = min(y_overlay_offset + sub.shape[0], tmp.shape[0])
        y_delta = y_overlay_end - y_overlay_offset
        x_overlay_end = min(x_overlay_offset + sub.shape[1], tmp.shape[1])
        x_delta = x_overlay_end - x_overlay_offset
        # Insert the image in overlay location
        tmp[y_overlay_offset:y_overlay_end, x_overlay_offset:x_overlay_end] = sub[:y_delta,:x_delta]
        return cv_to_msg(tmp)
    ##

    def my_overlay ( img_1, img_2 ):

        s_img = img_1
        l_img = img_2
        x_offset=y_offset=0
        l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1]] = s_img

        return cv2_to_msg ( l_img )

    def img_from_txt ( text ):

        # On prépare le texte
        image = Image.new("RGB", (1024,600), (255,255,255))
        usr_font = ImageFont.truetype("/home/baxter/ros_ws/src/projet_rob4/resources/fonts/Aller_Rg.ttf", TEXT_size)
        d_usr = ImageDraw.Draw(image)
        d_usr = d_usr.text((POS_x,POS_y), text, (0,0,0), font=usr_font)

        return gen_cv ( image )

    def republish(imagemsg):

        # Superposer l'image texte
        msg3 = my_overlay ( msg_to_cv ( cv_to_msg( img_from_txt ( my_text ) ) ) , msg_to_cv(imagemsg) )

        """
            Sends the camera image to baxter's display
        """
        display_pub.publish(msg3)
        rospy.sleep(0.1)


    # Met à jour le texte à afficher
    def texteCallback ( msg ):
        global my_text
        my_text = msg.data

    rospy.init_node('afficheur')      

    # Lis les messages sur le flux rob4/out et les affiche à l'écran
    out_sub = rospy.Subscriber ( TOPIC_OUT, String, texteCallback, None, 1 )
    cam_sub = rospy.Subscriber( "image_raw", ImageMsg, republish, None, 1)
    display_pub = rospy.Publisher('/robot/xdisplay', ImageMsg, queue_size=1) 
    
    rospy.spin()

if __name__ == '__main__':
    main()
