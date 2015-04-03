#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

from utils import *

from std_msgs.msg import String

# TODO A changer
from test_rob4.msg import Deplacement

clavier_deplacement_bindings = {
    'z': ( CMD_UP ),
    's': ( CMD_DOWN ),
    'q': ( CMD_LEFT ), 
    'd': ( CMD_RIGHT ),
    '+': ( CMD_IN ),
    '-': ( CMD_OUT ),
}

def main():



    def usage():

        output ( "Dplcmnts[ZQSD+-], [R]églages, Quitter[Esc], Stop[autres]" )

    def boucle():

        done = False;

        while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                
                output_msg = ""
                                
                # Déplacements
                if c in clavier_deplacement_bindings:
                   
                    output_msg = String ( clavier_deplacement_bindings[ c ] )

                # Réglages
                
                # Quitter
                elif c in ['\x1b', '\x03']:
                    done = True
                
                # Stop
                else:
                    usage()
                    output_msg = String ( CMD_STOP )
                    
                intr_pub.publish ( output_msg )



    rospy.init_node('interpreteur_clavier')

    intr_pub = rospy.Publisher ( TOPIC_CMD_TXT, String )


    usage()

    boucle()



if __name__ == '__main__':
    main()
