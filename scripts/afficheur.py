#!/usr/bin/env python

import os
import sys
import argparse

import rospy

import cv
import cv_bridge

import roslib
#import baxter_interface

#from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import (
    Image,
)
		
		

def main():

    rospy.init_node('afficheur')

    def send_image(path):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        img = cv.LoadImage(path)
        msg = cv_bridge.CvBridge().cv_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.spin()

    send_image("/home/baxter/ros_ws/wall_e.jpg")

if __name__ == '__main__':
	main()
