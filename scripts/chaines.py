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

TOPIC_RPR_INSTRU        = TOPIC_CONFIG_NAMESPACE + "rpr_instru"
TOPIC_INSTRU            = TOPIC_BASE_NAMESPACE + "instru_pose"


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
#     *** Messages ***
#
BAXTER_READY = "I am ready"
BAXTER_NO_TROCAR = "No trocar is set"


#
#     *** Bindings ***
#
deplacement_bindings = {
	CMD_UP		: ( [ 0, -1 ] ),
	CMD_DOWN 	: ( [ 0, 1 ] ),
	CMD_LEFT	: ( [ 1, +1 ] ),
	CMD_RIGHT	: ( [ 1, -1 ] ),
	CMD_IN		: ( [ 2, +2 ] ), # IN
	CMD_OUT		: ( [ 2, -4 ] )  # OUT
}
