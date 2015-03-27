#!/usr/bin/python
# -*- coding: utf8 -*-

import rospy
from std_msgs.msg import String

output_pub = rospy.Publisher ( 'rob4/out', String )


def output ( txt ):
    output_msg = String ( txt )
    out_pub = rospy.Publisher ( 'rob4/out', String )
    output_pub.publish ( output_msg )


def log ( obj_or_str ):

    if isinstance ( obj_or_str, basestring ):
        print obj_or_str.encode('utf8')
    else:
        print str ( obj_or_str ).encode ( 'utf8' )

