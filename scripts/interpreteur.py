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

def main ( ):
    
    # Publie le message de type Deplacement correspondant à la commande recue
    def envoyerDeplacement ( commandes ):
        parametres = deplacement_bindings [ commandes[ 0 ] ]
        deplacement_message = Deplacement ()
        deplacement_message.axe = parametres[ 0 ]
        deplacement_message.sens = parametres[ 1 ]
        cmd_pub.publish ( deplacement_message )
        
    
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
            
            #elif commandes[ 0 ] in reglages_bindings:
            
            elif commandes[ 0 ] == CMD_STOP:
                
                envoyerStop ( )

    
    
    # 
    #    *** Main ***
    #
    
    rospy.init_node( "interpreteur" )

    cmd_txt_sub = rospy.Subscriber ( TOPIC_CMD_TXT, String, commandeCallback )
    
    cmd_pub = rospy.Publisher ( TOPIC_DEPLACEMENT, Deplacement )

    rospy.spin()

if __name__ == '__main__':
    main()
    
    
