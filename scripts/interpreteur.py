#!/usr/bin/python
# -*- coding: utf8 -*-

#
# Ce fichier lit les chaines String sur l'entrée txt et les 
# convertit en commande compréhensible par les autres 
# noeuds
#

import rospy

from utils import *
from chaines import *
from valeurs import *

from projet_rob4.msg import Deplacement
from projet_rob4.msg import Instrument

# La calibration de Sarah
from calibration import calibration

def main ( ):
    
    # Publie le message de type Deplacement correspondant à la commande recue
    def envoyerDeplacement ( commandes ):
        parametres = deplacement_bindings [ commandes[ 0 ] ]
        deplacement_message = Deplacement ()
        deplacement_message.axe = parametres[ 0 ]
        deplacement_message.sens = parametres[ 1 ]
        cmd_pub.publish ( deplacement_message )


    # Possibilités de configuration du robot
    def configurationRobot ( commandes ):

        if ( commandes[ 1 ] == CMD_INSTRU ):

            msg = Instrument()
            
            if ( commandes[ 2 ] == CMD_INSTRU_1 ):
                code = 1
            elif ( commandes[ 2 ] == CMD_INSTRU_2 ):
                code = 2
            elif ( commandes[ 2 ] == CMD_INSTRU_3 ):
                code = 3
            else:
                code = CODE_NO_INSTRU
            
            msg.code_orientation = code

            instru_pub.publish ( msg )

        elif ( commandes[ 1 ] == CMD_TROCART ):

            calibration( baxter_interface.Limb( BRAS_UTILISE ) )

            output ( "Calibration terminee" )
        
    
    #
    #    *** Callbacks
    #
    # Gère la reception d'un message de commande sur le topic CMD_TXT
    def commandeCallback ( message ):
        # on lit le message sur la commande texte
        commandes = message.data.split(' ')

        if len( commandes ) != 0:
        
            # on le compare aux commandes connues
            # et on envoie la commande correspondante sur le topic approprié
            if commandes[ 0 ] in deplacement_bindings:
                
                envoyerDeplacement ( commandes )
            
            elif commandes[ 0 ] == CMD_CONF:

                configurationRobot ( commandes )
            
            elif commandes[ 0 ] == CMD_STOP:
                
                envoyerStop ( )

    
    
    # 
    #    *** Main ***
    #
    
    rospy.init_node( "interpreteur" )

    cmd_txt_sub = rospy.Subscriber ( TOPIC_CMD_TXT, String, commandeCallback )
    
    cmd_pub = rospy.Publisher ( TOPIC_DEPLACEMENT, Deplacement )
    instru_pub = rospy.Publisher ( TOPIC_INSTRU, Instrument )

    rospy.spin()

if __name__ == '__main__':
    main()
    
    
