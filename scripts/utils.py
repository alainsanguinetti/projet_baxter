#!/usr/bin/python
# -*- coding: utf8 -*-

import rospy
import numpy as np
import ik_command

import baxter_interface

from std_msgs.msg import String
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import Quaternion
from cgkit.cgtypes      import * # Pour les quaternions

# Nos fichiers communs
from chaines import *
from valeurs import *
import instru

# Le topic pour afficher sur l'écran de Baxter
output_pub = rospy.Publisher ( TOPIC_OUT, String )
# Le topic de stop
stop_pub = rospy.Publisher ( TOPIC_STOP, String )


# ######################################################################
# #
# #				Log et prints                                  
# #
# ######################################################################
# Convert any python arg to its string representation
def convert_to_string ( obj_or_str ):
    if isinstance ( obj_or_str, basestring ):
        return obj_or_str.encode('utf8')
    else:
        return str ( obj_or_str ).encode ( 'utf8' )

# Publish something on baxter's screen
def output ( txt ):
    string = convert_to_string ( txt )
    output_msg = String ( string )
    out_pub = rospy.Publisher ( 'rob4/out', String )
    output_pub.publish ( output_msg )


# Print something ( when using roslaunch, it goes into a log )
def log ( obj_or_str ):

    print convert_to_string ( obj_or_str )


# ######################################################################
# #
# #				Quaternions, point, Vec3 et consorts                                  
# #
# ######################################################################

# Les quat de cgtype sont dans l'ordre w, x, y, z
# les quats ROS sont dans l'ordre x, y, z, w
def quatFromOrientation ( orientation ):

    return quat ( orientation.w,
                  orientation.x,
                  orientation.y,
                  orientation.z )

def quaternionFromQuat ( quat ):

    return Quaternion ( quat.x, quat.y, quat.z, quat.w )

# L'inverse de la fonction quatFromOrientation
def ecrirePose ( quat_souhaite ):

    mat_quaternion = np.zeros ( 7 )

    mat_quaternion[ 3 ] = quat_souhaite.x
    mat_quaternion[ 4 ] = quat_souhaite.y
    mat_quaternion[ 5 ] = quat_souhaite.z
    mat_quaternion[ 6 ] = quat_souhaite.w

    return mat_quaternion

# Faire une pose à partir d'une position et d'une orientation
def poseFromPointQuat ( point, quat ):
    
    mat = np.zeros( 7 )
    mat[ 0 ] = point.x
    mat[ 1 ] = point.y
    mat[ 2 ] = point.z
    mat[ 3 ] = quat.x
    mat[ 4 ] = quat.y
    mat[ 5 ] = quat.z
    mat[ 6 ] = quat.w

    return mat
    

# Return the sum of two points
def addPoints ( point1, point2 ):
    
    return Point ( point1.x + point2.x,
                    point1.y + point2.y,
                    point1.z + point2.z )

# Return a matrix that describes the pose
def matriceFromPose ( pose ):
    
    matrice = np.zeros ( 7 )
    
    matrice[0] = pose.position.x
    matrice[1] = pose.position.y
    matrice[2] = pose.position.z
    matrice[3] = pose.orientation.x
    matrice[4] = pose.orientation.y
    matrice[5] = pose.orientation.z
    matrice[6] = pose.orientation.w

    return matrice

# Returns a vec3 from a point
def vec3FromPoint ( point ):
    
    return vec3 ( point.x,
                    point.y,
                    point.z )

# Returns a point from a vec3
def pointFromVec3 ( vec3 ):
    
    return Point ( vec3.x,
                    vec3.y,
                    vec3.z )

# Returns the vector [AB]
def vec3FromPoints ( B, A ):
    
    return vec3 ( B.x - A.x, B.y - A.y, B.z - A.z )

# ######################################################################
# #
# #				Déplacements                                  
# #
# ######################################################################

# Appliquer une demande de nouvelle Pose en utilisant le service de ik_command
def appliquerDeplacement ( pose_souhaitee, limb, limb_hndle ):

    iksvc, ns = ik_command.connect_service( limb )

    log ( "Pose actuelle :" )
    log ( limb_hndle.endpoint_pose() )

    log ( "Nouvelle pose :" )
    log ( pose_souhaitee )

    ik_command.service_request(iksvc, pose_souhaitee, limb )
    

# Calcule l'erreur entre 2 pose, la première est une matrice, la 2eme est un objet Pose
def erreurFromPoses ( pose, pose_actu ):
    
    # pose est une matrice
    # pose_actu est un Pose
    pose_actu = matriceFromPose ( pose_actu )
    # on renvoie une matrice
    erreur = pose - pose_actu

    log ( "MAtrice d'erreur de position" )
    log ( erreur )

    return erreur
    

# Asservir la position du poignet à l'aide d'un intégrateur
def asservirPoignet ( pose, limb, limb_hndle ):
    
    KI = 0.5
    pose_souhaitee = pose

    i=0

    # On fait au max 10 essais
    while i < 20:
        
        output ( "Essai " + str ( i ) )
        # On déplace le poignet - avec la pose souhaitée
        appliquerDeplacement ( pose_souhaitee, limb, limb_hndle )

        # On mesure la pose atteinte
        pose_actu = Pose ( limb_hndle.endpoint_pose()[ 'position' ], limb_hndle.endpoint_pose()[ 'orientation' ] )
        
        # On calcule l'erreur - entre la commande et la position actuelle
        pose_erreur = erreurFromPoses ( pose, pose_actu )
        
        # Si l'erreur est suffisamment faible, on s'arrête
        if ( np.linalg.norm ( pose_erreur ) < ( 0.2 * DEPLACEMENT ) ):
            
            break
        
        # Sinon, on ajoute une proportion de l'erreur à la commande ( on peut limiter la norme du déplacement max demandé )
        else:
            
            pose_souhaitee = pose_souhaitee + KI * pose_erreur
            i = i + 1
        
        # Et c'est reparti !


# ######################################################################
# #
# #				Gestion du stop                                  
# #
# ######################################################################

# Publie le message de stop
def envoyerStop():
        
        stop_pub.publish ( String ( ) )

# ######################################################################
# #
# #				Lecture de la pose de l'instrument                                  
# #
# ######################################################################

rpr_instru = Pose ()
instru.rpr = Pose()

# Met à jour le repère instrument
def setInstruCallback ( rpr ):

    instru.rpr = rpr

instru_sub = rospy.Subscriber ( TOPIC_RPR_INSTRU, Pose, setInstruCallback )

# Renvoie une Pose contenant la position et l'orientation de l'outil dans le repère monde
def pose_instrument ( ):

    limb_hndle = baxter_interface.Limb ( BAXTER_USED_LIMB )

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
                            vec3FromPoint ( instru.rpr.position ) 
                            )
                        )

    position = addPoints ( position_poignet, decalage_outil )

    orientation = quaternionFromQuat ( 
                                        quatFromOrientation ( instru.rpr.orientation ) * 
                                        quatFromOrientation ( limb_hndle.endpoint_pose()['orientation'] )
                                    )

    return Pose ( position, orientation )

    


