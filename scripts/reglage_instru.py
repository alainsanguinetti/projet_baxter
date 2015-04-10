#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices
import numpy as np

import sys

from math import pi

from cgkit.cgtypes import * # Pour les quaternions
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

# Nos librairies et fonctions de configuration
from chaines import *
from valeurs import *

def main ( ):

    # Création du noeud
    rospy.init_node ( 'reglage_instru' )

    # Charger une configuration
    config = ( 0.16, 0, 0.3, 0, 0, 0 )
    config = ( 0.23, 0, 0.3 )
    config = ( 0.3, 0, 0.3 )

    vecteur_rota_x = vec3 ( 1, 0, 0 ) # rotation de pi / 2 selon x
    vecteur_rota_y = vec3 ( 0, 1, 0 )
    vecteur_rota_z = vec3 ( 0, 0, 1 )
    rotation1 = quat().fromAngleAxis ( pi/3, vecteur_rota_y )
    rotation2 = quat().fromAngleAxis ( 1/3, vecteur_rota_z )
    #rotation = rotation1 * rotation2
    rotation = rotation1
    rotation = quat().fromAngleAxis ( pi/2, vecteur_rota_y )

    # Préparer le message à envoyer
    point = Point ( config[0], config[1], config[2] )
    orientation = Quaternion ( rotation.x, rotation.y, rotation.z, rotation.w )
    msg = Pose ( point, orientation )


    # Envoyer le message sur le topic
    pub = rospy.Publisher ( TOPIC_RPR_INSTRU, Pose, latch=True )

    pub.publish ( msg )

    # Sinon, la publication n'est pas prise en compte
    rospy.spin()



if __name__ == '__main__':
    main()
