#!/usr/bin/python
# -*- coding: utf8 -*-

from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

from std_msgs.msg import UInt16

def main():
    
    rospy.init_node('extinction_sonar')

    rospy.Publisher('/robot/sonar/head_sonar/set_sonars_enabled', UInt16, latch=True ).publish( 0 )

    rospy.sleep ( 0.5 )

if __name__ == '__main__':
    main()
