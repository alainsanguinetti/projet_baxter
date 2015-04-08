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


ctr_trocart = Point ()  # Variables globales
trocartIsSet = False
rpr_outil = Pose ()
limb = ''


# La fonction main
def main():

    def decalerInstrument ( pos_outil, sens, limb_hndle ):
        # On calcule la matrice de déplacement dans le repère instru
        dep_instru = vec3 ( 0, 0, sens * DEPLACEMENT )

        # on l'exprime dans le repère poignet
        dep_instru = quatFromOrientation ( rpr_outil.orientation ).rotateVec ( dep_instru )
        
        # Puis on l'exprime dans le repère cartésien
        dep_poignet = quatFromOrientation ( limb_hndle.endpoint_pose()['orientation'] ).rotateVec ( dep_instru )

        # on additione avec la position actuelle du poignet
        # et paf, ça fait une nouvelle pose
        orientation = limb_hndle.endpoint_pose()['orientation']
        position = addPoints ( limb_hndle.endpoint_pose()['position'], pointFromVec3 ( dep_poignet ) )

        log ( "On décale l'instrument" )

        return poseFromPointQuat( position, orientation ) 
                            

    def poseFuturePoignet ( pos_outil, pos_outil_f, limb_hndle ):
       # Calcul de l'axe actuel
        vec_axe_initial = vec3FromPoints ( pos_outil, ctr_trocart )

        # Calcul de l'axe futur
        vec_axe_final = vec3FromPoints ( pos_outil_f, ctr_trocart )

        # Calcul d'un vecteur normal
        vec_rotation = vec_axe_final.cross( vec_axe_initial )

        # Calcul de la rotation
        rad_rotation = vec_axe_final.angle( vec_axe_initial )

        # Calcul du quaternion correspondant
        quat_rotation = quat().fromAngleAxis( 
                            2 * pi -  rad_rotation, vec_rotation 
                        )

        # Calcul de la nouvelle orientation
        quat_souhaite = quat_rotation * quatFromOrientation (
                            limb_hndle.endpoint_pose()['orientation']
                        )

        # Calcul de la nouvelle position
        # Decalage outil ( repère poignet ) exprimé dans le repère cartésien
        decalage_outil = pointFromVec3 ( 
                            quat_souhaite
                            .rotateVec ( 
                                vec3FromPoint ( rpr_outil.position ) 
                                )
                            )

        # La nouvelle position est la suivante :
        position_souhaitee = addPoints ( pos_outil_f, Point ( 
                                                        - decalage_outil.x,
                                                        - decalage_outil.y,
                                                        - decalage_outil.z
                                                        )
                                        )                  



        return poseFromPointQuat ( position_souhaitee, quat_souhaite )


    # Calcule la position cartésienne future de l'outil
    def positionFuture ( pos_outil, axe_nb, sens, limb_hndle ):

        # calculer le déplacement dans le repère outil
        mat_deplacement_loc = np.zeros( 3 )
        mat_deplacement_loc[ axe_nb ] += sens * DEPLACEMENT
        vect_deplacement_local = vec3 ( mat_deplacement_loc )

        log ("Vecteur du déplacement local")
        log (vect_deplacement_local)

        # exprimer le déplacement dans le repère poignet
        quat_outil_poignet = quatFromOrientation ( rpr_outil.orientation )
        vect_deplacement_poignet = quat_outil_poignet.rotateVec( vect_deplacement_local )
        log("Vecteur du déplacement dans le repère poignet")
        log( vect_deplacement_poignet )

        # exprimer le déplacement dans le repère cartésien
        quat_poignet_monde = quatFromOrientation ( limb_hndle.endpoint_pose()[ 'orientation' ] )
        vect_deplacement_cart = quat_poignet_monde.rotateVec( vect_deplacement_poignet )
        point_dep_cart = Point ( vect_deplacement_cart.x,
                                    vect_deplacement_cart.y,
                                    vect_deplacement_cart.z )
        log ( "Vecteur du déplacement dans le repère cartésien" )
        log ( vect_deplacement_cart )

        return addPoints ( point_dep_cart, pos_outil )
    
    # Calcule la position future du poignet
    def positionFuturePoignet ( pos_outil, pos_outil_f, limb_hndle ):
        
        # Vecteur outil-trocart
        vect_outil_trocart = vec3 ( ctr_trocart.x - pos_outil.x, 
                                    ctr_trocart.y - pos_outil.y,
                                    ctr_trocart.z - pos_outil.z )

        # Vecteur poignet-trocart
        pos_poignet = limb_hndle.endpoint_pose()[ 'position' ]
        vect_poignet_trocart = vec3 ( ctr_trocart.x - pos_poignet.x,
                                        ctr_trocart.y - pos_poignet.y,
                                        ctr_trocart.z - pos_poignet.z )

        # position par rapport au trocart ( dedans / dehors )
        # TODO
        sens = -1

        # position du poignet future
        vect_outil_trocart = vec3FromPoints ( 
                                ctr_trocart, 
                                pos_outil_f 
                                ).normalize()

        pos_poignet_f = addPoints ( 
                            pos_outil_f,
                            pointFromVec3 (
                                sens * 
                                vec3FromPoint ( 
                                    rpr_outil.position
                                ).length() *
                                vect_outil_trocart)
                            )
                        

        return pos_poignet_f
        

    # Donne la position actuelle du point piloté de l'outil dans le repère cartésien du robot
    def recupererPositionOutil ( limb_hndle ):
        
        # Position du poignet
        position = limb_hndle.endpoint_pose()[ 'position' ]
        position_poignet = Point ( position.x, position.y, position.z )

        # Decalage outil ( repère poignet ) exprimé dans le repère cartésien
        decalage_outil = pointFromVec3 ( 
                            quatFromOrientation( 
                                limb_hndle.endpoint_pose()['orientation'] 
                                )
        #                    .inverse ()
                            .rotateVec ( 
                                vec3FromPoint ( rpr_outil.position ) 
                                )
                            )

        log ( "Décalage de l'outil exprimé dans le repère cartésien" )
        log ( decalage_outil )

        position_outil = addPoints ( position_poignet, decalage_outil )
        log ( "Position de l'outil actuelle dans le repère cartésien" )
        log ( position_outil )

        return position_outil


    # Calculer la nouvelle Pose correspondant au déplacement souhaité
    def calculerDeplacement ( axe_nb, sens, limb_hndle ):

        # Recuperer la position cartésienne actuelle de l'outil
        pos_outil = recupererPositionOutil ( limb_hndle )

        # Si on se déplace sur l'axe de l'instrument
        if ( axe_nb == 2 ):

            # On a juste à décaler la pos du poignet
            pose_souhaitee = decalerInstrument ( pos_outil, sens, limb_hndle )


        # Si on déplace le bout de l'instrument
        else:
            # Calculer la position cartésienne future de l'outil
            pos_outil_f = positionFuture ( pos_outil, axe_nb, sens, limb_hndle )
            
            # Calculer la position cartésienne et l'orientation future du poignet
            pose_souhaitee = poseFuturePoignet ( pos_outil, pos_outil_f, limb_hndle )

        return pose_souhaitee

  
    #
    #					*** Callbacks ***
    #
    
    # Fait une pause pendant 3 secondes
    def stopCallback ( msg ):

        output ( CMD_STOP )
        
        rospy.sleep ( 3 )
        

    # Calcule et effectue un déplacement
    def deplacement ( commande ):

        log ("Demande de déplacement reçue")
    
        if trocartIsSet :

            axe_nb = commande.axe
            sens = commande.sens

            limb_hndle = baxter_interface.Limb( limb )

            asservirPoignet ( calculerDeplacement ( axe_nb, sens, limb_hndle ), limb, limb_hndle )

            output ( commande )
        
        else:
            
            output ( "No trocart is set" )
            sys.stdout.flush()

    # Met à jour la position du trocart 
    def setTrocartCallback ( coordonnees ):

        global ctr_trocart
        ctr_trocart = coordonnees

        global trocartIsSet
        trocartIsSet = True

        output ( "Trocart updated" )

        return

    # Met à jour le repère d'outil
    def setOutilCallback ( new_rpr ):
        
        global rpr_outil
        rpr_outil = new_rpr

        log( "Repere d'outil mis à jour" )
        sys.stdout.flush ()

    #
    #           *** La boucle d'écoute principale, on ne s'occupe pas des caractères entrés au clavier car on est dans un roslaunch ***
    # 
    def boucle( limb ):

        done = False

        rospy.spin()

        return


    #
    #           *** Lancement de la commande ***
    #
    rospy.init_node('calculateur_rpr_outil')

    global limb
    limb = 'right'

    trocart_sub = rospy.Subscriber( 'api_rob4/config/ctr_trocart', Point, setTrocartCallback )
    cmd_sub = rospy.Subscriber( TOPIC_DEPLACEMENT, Deplacement, deplacement )
    outil_sub = rospy.Subscriber( 'api_rob4/config/rpr_outil', Pose, setOutilCallback )
    stop_sub = rospy.Subscriber ( TOPIC_STOP, String, stopCallback )

    boucle ( limb )



if __name__ == '__main__':
    main()


