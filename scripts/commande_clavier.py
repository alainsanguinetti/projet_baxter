#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

from utils import *

from std_msgs.msg import String

# A changer
from test_rob4.msg import Deplacement

clavier_bindings = {
	'z': ( [ 0, -1 ] ),
	's': ( [ 0, 1 ] ),
	'q': ( [ 1, +1 ] ),
	'd': ( [ 1, -1 ] ),
	'+': ( [ 2, +1 ] ), # IN
	'-': ( [ 2, -4 ] )  # OUT
}


def main():



    def usage():

	    output ( "Dplcmnts[ZQSD+-], [R]églages, Quitter[Esc], Stop[autres]" )

    def boucle():

        done = False;

        while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:                
                # Déplacements
                if c in clavier_bindings:

                    parametres = clavier_bindings [ c ]
                    output_msg = "%c: selection de l'axe %d, sens %d" % ( c, parametres[ 0 ], parametres[ 1 ] )
                    deplacement_message = Deplacement ()
                    deplacement_message.axe = parametres[ 0 ]
                    deplacement_message.sens = parametres[ 1 ]
                    cmd_pub.publish ( deplacement_message )
                    output ( output_msg )

                # Réglages
                
                # Quitter
                elif c in ['\x1b', '\x03']:
                    done = True
                
                # Stop
                else:
                    usage()


    rospy.init_node('interpreteur_clavier')

    cmd_pub = rospy.Publisher( 'api_rob4/commande', Deplacement )


    usage()

    boucle()



if __name__ == '__main__':
    main()
