#!/usr/bin/env python
# -*- coding: utf8 -*-

import os
import sys
import argparse

import rospy

import numpy as np

import cv
import cv_bridge
import PIL

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



def main():

    POS_x = 60
    POS_y = 10
    TEXT_size = 60
    text = "  "

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

    def republish(imagemsg):

        # On récupère l'image actuelle
        
        # L'image de fond
        cam_img = msg_to_cv ( imagemsg )

        # On prépare le texte
        image = Image.new("RGB", (500,100), (255,255,255))
        usr_font = ImageFont.truetype("/home/baxter/ros_ws/src/projet_rob4/resources/fonts/Aller_Rg.ttf", TEXT_size)
        d_usr = ImageDraw.Draw(image)
        d_usr = d_usr.text((POS_x,POS_y), text, (0,0,0), font=usr_font)

        txt_msg = gen_cv ( image )

        msg = overlay ( cam_img, txt_msg, 0, 0 )

        """
            Sends the camera image to baxter's display
        """        
        display_pub.publish(msg)

    # Met à jour le texte à afficher
    def texteCallback ( msg ):

        text = msg.data


    rospy.init_node('afficheur')      

    # Lis les messages sur le flux rob4/out et les affiche à l'écran
    out_sub = rospy.Subscriber ( TOPIC_OUT, String, texteCallback )
    cam_sub = rospy.Subscriber( "image_raw", ImageMsg, republish, None, 1)
    pub = rospy.Publisher('/robot/xdisplay', ImageMsg, queue_size=1) 

    rospy.spin()

if __name__ == '__main__':
	main()
