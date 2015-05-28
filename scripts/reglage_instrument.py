#!/usr/bin/python
# -*- coding: utf8 -*-
# Alain Sanguinetti 2015-05-15
# Permet de choisir un réglage de position d'instrument parmi les 3 positions possibles

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

# Pour les transformations
import transformations as trf

from baxter_core_msgs.msg import EndEffectorCommand

# Nos librairies et fonctions de configuration
from chaines    import *
from utils      import *

# Le format de message de choix d'un instrument
from projet_rob4.msg import Instrument

# Les configurations possibles
# Chaque configuration est enregistrée sous la forme [ X, Y, Z, r, p, y ]
# Les angles d'euler en notation XYZ
angles = {
    1 : ( [ 0, 0.96, 0 ] ),                 # Rotation de 35° autour de Y0
    2 : ( [ 0, 0.96, 4.188790205 ] ),       # rotation de 4pi/3 autour de Z1
    3 : ( [ 0, 0.96, 2.094395102 ] ),       # + rotation de 2pi/3 autour de Z1
}

for i in range (1,4):
    mat = trf.euler_matrix ( angles[ i ][0], angles [ i ][ 1 ], angles[ i ][ 2 ], 'rxyz' );
    al, be, ga = trf.euler_from_matrix ( mat, 'rzyx' )          # Euler ZYX <=> RPY
    angles[ i ][ 0 ] = al
    angles[ i ][ 1 ] = be
    angles[ i ][ 2 ] = ga

table = {
    0 : ( [ 0.0, 0.0, 0.025, 0, 0, 0 ] ),               # Position initiale
    1 : ( [ 0.225, 0.0, 0.32, angles[1][2], angles[1][1], angles[1][0] ] ), 
    2 : ( [ 0.225, 0.0, 0.32, angles[2][2], angles[2][1], angles[2][0] ] ),           # Rotation de 35° de l'instrument
    3 : ( [ 0.225, 0.0, 0.32, angles[3][2], angles[3][1], angles[3][0] ] ),         
}

#    1 : ( [ 0.265, 0.0, 0.165, 0.0, 1.570796327, 0.0 ] ), # ( Rotation de pi / 2 selon y, l'instrument est à 90° de l'axe du poignet )


def main ( ):

    # Publie le message qui permet de changer le repère d'instrument
    def changementRepere ( msg ):

        # Lire l'orientation souhaitée
        code = msg.code_orientation
        params = table [ code ]
        
        # Préparer la commande à envoyer à Baxter
        args = " { \"urdf\":{" \
                        "\"name\": \"right_electric_gripper\"," \
                        "\"joint\": [ " \
                            "{ \"name\": \"right_hand\" }," \
                            "{ \"name\": \"right_gripper_base\", \"origin\": { " \
                                        "\"xyz\": [ " + \
                                                    convert_to_string(params[0]) + ", " + \
                                                    convert_to_string(params[1]) + ", " + \
                                                    convert_to_string(params[2]) + " ], "\
                                        "\"rpy\" : [ " + \
                                                    convert_to_string(params[3]) + ", " + \
                                                    convert_to_string(params[4]) + ", " + \
                                                    convert_to_string(params[5]) + " ] " + \
                            "} }" \
                        "]" \
                 "} }" \

        log ( args.encode('ascii') )
        
        msg = EndEffectorCommand( 
                    EFFECTOR_ID, 
                    EndEffectorCommand.CMD_CONFIGURE, 
                    args.encode('ascii'), 
                    'alain'.encode('ascii'),
                    0)

        # Envoyer la commande
        pub.publish ( msg )

        # Dire qu'on a terminé
        output ( "Instrument : reglage " + convert_to_string ( code ) )

    # Creation du noeud
    rospy.init_node ( 'reglage_instrument' )

    # Creation de l'abonné sur le topic de demande de modification d'instrument
    sub = rospy.Subscriber ( TOPIC_INSTRU, Instrument, changementRepere )

    # Creation du publicateur sur le topic de commande URDF
    pub = rospy.Publisher ( "/robot/end_effector/right_gripper/command", EndEffectorCommand )

    # Et on tourne en boucle !
    rospy.spin()

if __name__ == '__main__':
    main()

    
