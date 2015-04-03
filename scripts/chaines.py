#!/usr/bin/python
# -*- coding: utf8 -*-

#
# Ce fichier définit les chaines de caractères utilisées dans le projet 
# Baxter de Sara, Eder, Alain
#

#
#     *** Topics ***
#
TOPIC_BASE_NAMESPACE = "rob4/"

TOPIC_CONFIG_NAMESPACE	= TOPIC_BASE_NAMESPACE + "config/"

TOPIC_CTR_TROCART 		= TOPIC_CONFIG_NAMESPACE + "ctr_trocart"
TOPIC_RPR_OUTIL			= TOPIC_CONFIG_NAMESPACE + "rpr_outil"

TOPIC_OUT				= TOPIC_BASE_NAMESPACE + "out"

TOPIC_STOP				= TOPIC_BASE_NAMESPACE + "stop"

TOPIC_CMD_TXT 			= TOPIC_BASE_NAMESPACE + "cmd_txt"
TOPIC_DEPLACEMENT 		= TOPIC_BASE_NAMESPACE + "deplacement"


#
#     *** Commandes ***
#
CMD_UP 		= "UP"
CMD_DOWN 	= "DOWN"
CMD_LEFT 	= "LEFT"
CMD_RIGHT 	= "RIGHT"
CMD_IN		= "IN"
CMD_OUT		= "OUT"

CMD_STOP 	= "STOP"



#
#     *** Bindings ***
#
deplacement_bindings = {
	CMD_UP		: ( [ 0, -1 ] ),
	CMD_DOWN 	: ( [ 0, 1 ] ),
	CMD_LEFT	: ( [ 1, +1 ] ),
	CMD_RIGHT	: ( [ 1, -1 ] ),
	CMD_IN		: ( [ 2, +1 ] ), # IN
	CMD_OUT		: ( [ 2, -4 ] )  # OUT
}
