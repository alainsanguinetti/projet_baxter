#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices
import numpy as np

import sys

from math import pi

from cgkit.cgtypes      import * # Pour les quaternions et vec3
from test_rob4.msg      import Deplacement
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import Quaternion

# Nos librairies et fonctions de configuration
from utils import *	
from chaines import *
from valeurs import *

def main ():

    # Tant qu'on est pas "shutdown", on publie la position de l'instrument sur le topic
    rospy.init_node ( "pose_instrument" )

    instru_pub = rospy.Publisher ( TOPIC_INSTRU, Pose, latch=False )
    
    rate = rospy.Rate ( 10 )

    while not rospy.is_shutdown():

        instru_pub.publish ( pose_instrument() )

        rate.sleep()

if __name__ == '__main__':
    main()
