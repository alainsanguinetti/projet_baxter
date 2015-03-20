#!/usr/bin/python
# -*- coding: utf8 -*-
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

from std_msgs.msg import String

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

		print ( "Z = up, S = down, Q = left, D = right, + = in, - = out" )

	def boucle():

		done = False;

		while not done and not rospy.is_shutdown():
			c = baxter_external_devices.getch()
			if c:
				#catch Esc or ctrl-c
				if c in ['\x1b', '\x03']:

					 done = True
					 rospy.signal_shutdown("Example finished.")

				elif c in clavier_bindings:

					parametres = clavier_bindings [ c ]
					print( "%c: selection de l'axe %d, sens %d" % ( c, parametres[ 0 ], parametres[ 1 ] ) )
					deplacement_message = Deplacement ()
					deplacement_message.axe = parametres[ 0 ]
					deplacement_message.sens = parametres[ 1 ]
					cmd_pub.publish ( deplacement_message )

				else:

					 print("key bindings: ")
					 print("  Esc: Quit")
					 print("  ?: Help" )


	rospy.init_node('interpreteur_clavier')

	cmd_pub = rospy.Publisher( 'api_rob4/commande', Deplacement )

	usage()

	boucle()



if __name__ == '__main__':
    main()
