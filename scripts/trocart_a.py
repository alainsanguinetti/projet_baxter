#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

import rospy
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose
from cgkit.cgtypes      import * # Pour les quaternions

from utils import *
from chaines import *
import instru


# Fonctions

# Publier la position du point piloté sur le topic de centre de trocart
def publierPosition ( limb, pub ):

    coordonnees = limb.endpoint_pose()['position']

    message = Point ( coordonnees.x, coordonnees.y, coordonnees.z )

    print ( "Publication du message : " )
    print message

    pub.publish ( message )


# Donne la position actuelle du point piloté de l'outil dans le repère cartésien du robot
def recupererPositionOutil ( limb_hndle ):
  
    # Position du poignet
    position_poignet = Point ( limb_hndle.endpoint_pose()[ 'position' ].x,
                                 limb_hndle.endpoint_pose()[ 'position' ].y,
                                 limb_hndle.endpoint_pose()[ 'position' ].z )

    # Decalage outil ( repère poignet ) exprimé dans le repère cartésien
    decalage_outil = pointFromVec3 ( 
                         quatFromOrientation( 
                             limb_hndle.endpoint_pose()['orientation'] 
                             )
                         .rotateVec ( 
                             vec3FromPoint ( instru.rpr.position ) 
                             )
                         )

    print "Position du poignet"
    print position_poignet

    print "Decalage de l'instrument en cartésien"
    print decalage_outil

    position_outil = addPoints ( position_poignet, decalage_outil )

    return position_outil

# Publier la position de l'outil
def publierPositionInstrument ( limb, pub ):

    coordonnees = recupererPositionOutil ( limb );

    message = Point ( coordonnees.x, coordonnees.y, coordonnees.z )

    print ( "Publication du message : " )
    print message

    pub.publish ( message )



def boucle ( r_limb, l_limb, pub ):

    # on attend une entrée de l'utilisateur
    while ( 1 ):

        choix = raw_input ( "[r droit, g gauche, i instrument ou q quitter]\nPublier la position du bras : " )

        # on quitte ?
        if ( choix == 'q' ):

            break

        # on publie la position du bras correspondant
        else:

            if ( choix == 'r' ):

                publierPosition ( r_limb, pub )

            elif ( choix == 'g' ):

                publierPosition ( l_limb, pub )

            elif ( choix == 'i' ):
                    # WARNING pour l'instant on n'utilise que le bras droit
                publierPositionInstrument ( r_limb, pub )

            else:
	            print "Pas compris "

    # on boucle

    print ( "Au revoir !" )




def main():

    # le noeud est lancé
    rospy.init_node('Publication_position_2')

    # on initialise les bras
    r_limb = baxter_interface.Limb('right')
    l_limb = baxter_interface.Limb('left')

    pub = rospy.Publisher( TOPIC_CTR_TROCART, Point, latch=True)

    boucle( r_limb, l_limb, pub )



if __name__ == '__main__':
    main()

