#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

from utils import *

from std_msgs.msg import String

clavier_deplacement_bindings = {
    'z': ( CMD_UP ),
    's': ( CMD_DOWN ),
    'q': ( CMD_LEFT ), 
    'd': ( CMD_RIGHT ),
    '+': ( CMD_IN ),
    '-': ( CMD_OUT ),
}

def main():

    def pub_stop ():

        envoyerStop ( )

        output ( "STOP" )
        output_msg = String ( CMD_STOP )
        intr_pub.publish ( output_msg )
        rospy.sleep ( 2 )

    def message_trocart ():

        msg_str = CMD_CONF + " " + CMD_TROCART
        msg = String ( msg_str )
        intr_pub.publish ( msg )

    def message_instru ( c ):

        if ( c == '1' ):

            code = CMD_INSTRU_1

        elif ( c == '2' ):
            
            code = CMD_INSTRU_2

        elif ( c == '3' ):
        
            code = CMD_INSTRU_3

        else:
            code = CMD_INSTRU_0

        msg_str = CMD_CONF + " " + CMD_INSTRU + " " + code
        msg = String ( msg_str )
        intr_pub.publish ( msg )

    def choix_orientation ():

        output ( "[123] choix, [0] RAZ" )

        while not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:

                print ( "Recu " + c + " au clavier" )
                
                if c in [ '1', '2', '3', '0' ]:

                    message_instru ( c )
                    
                # Stop
                else:
                    pub_stop ()

                break

    def choix_reglage ( ):

        output ( "[I] instru [T] trocart" )

        while not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                
                if c in [ 'i', 'I' ]:

                    choix_orientation ( )
                    
                elif c in [ 't', 'T' ]:
        
                    message_trocart ()
                    
                # Stop
                else:
                    pub_stop ()

                break



    def usage():

        output ( "[ZQSD+-] [R] [Esc] Stop[autres]" )

    def boucle():

        done = False;

        while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                
                                        
                # Déplacements
                if c in clavier_deplacement_bindings:
                   
                    output_msg = String ( clavier_deplacement_bindings[ c ] )
                    intr_pub.publish ( output_msg )

                # Réglages
                elif c in [ 'r', 'R' ]:

                    choix_reglage ( )

                # Quitter
                elif c in ['\x1b', '\x03']:
                    done = True

                    output ( "Au revoir" )
                
                # Stop
                else:
                    usage()
                    output_msg = String ( CMD_STOP )
                    
                



    rospy.init_node('interpreteur_clavier')

    intr_pub = rospy.Publisher ( TOPIC_CMD_TXT, String )


    usage()

    boucle()



if __name__ == '__main__':
    main()
