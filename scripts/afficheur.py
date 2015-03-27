#!/usr/bin/env python
# -*- coding: utf8 -*-

import os
import sys
import argparse

import rospy

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
    ##

    def displayCallBack ( to_disp ):

        # On récupère l'image actuelle
        
        # On prépare le texte
        image = Image.new("RGB", (1024,100), (255,255,255))
        usr_font = ImageFont.truetype("/home/baxter/ros_ws/src/projet_rob4/resources/fonts/Aller_Rg.ttf", TEXT_size)
        d_usr = ImageDraw.Draw(image)
        d_usr = d_usr.text((POS_x,POS_y), to_disp.data, (0,0,0), font=usr_font)

        msg = cv_to_msg ( gen_cv ( image ) )

        # On envoie le texte
        pub.publish ( msg )

    rospy.init_node('afficheur')      

    # Lis les messages sur le flux rob4/out et les affiche à l'écran
    sub = rospy.Subscriber ( 'rob4/out', String, displayCallBack )  
    pub = rospy.Publisher('/robot/xdisplay', ImageMsg, queue_size=1) 

    rospy.spin()

if __name__ == '__main__':
	main()
