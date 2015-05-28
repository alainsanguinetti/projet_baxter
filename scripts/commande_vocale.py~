#!/usr/bin/python
# -*- coding: utf8 -*-
# 2015-05-19 Eder Jimenez et Alain Sanguinetti
from __future__ import unicode_literals

import rospy
import baxter_interface
import baxter_external_devices

# Pour la souplesse d'utilisation du répertoire
import os

# Commande vocale
import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

# Gestion du signal d'interruption
import signal
import sys

# Fonctions et chaines communes
from utils import *
from chaines import *

from std_msgs.msg import String

voix_deplacement_bindings = {
    'UP': ( CMD_UP ),
    'DOWN': ( CMD_DOWN ),
    'LEFT': ( CMD_LEFT ), 
    'RIGHT': ( CMD_RIGHT ),
    'IN': ( CMD_IN ),
    'OUT': ( CMD_OUT ),
}

numbers_bindings = {

    'ONE' : ( CMD_INSTRU_1 ),
    'TWO' : ( CMD_INSTRU_2 ),
    'THREE' : ( CMD_INSTRU_3 ),
    'ZERO' : ( CMD_INSTRU_0 ),
}

sleeping = True
isSettings = False

def main ():

    global sleeping
    global isSettings

    # Callback pour le signal SIGINT
    def signal_handler(signal, frame):
        print('Exiting')
        sys.exit(0)
    

    # Traite les résultats partiels, si on entend stop, on agit directement
    # On affiche en minuscule pour montrer que ça réfléchit
    def partial_result( hyp, uttid):
        hyp = hyp.lower() 
        
        # On n'affiche uniquement si on est "réveillé"
        if not sleeping or "sleep" in hyp:
            output(  hyp )
        
        if "stop" in hyp:
            envoyerStop ()

    # Gère la réception de commande de déplacement,
    # Si une commande est reconnue, on envoie une commande à l'interpréteur
    def testerCommandesDeplacement ( hyp ):
        
        # Cas des commandes de déplacements
        if hyp in voix_deplacement_bindings:
            
            output_msg = String ( voix_deplacement_bindings[ hyp ] )
            intr_pub.publish ( output_msg )

            return True

    # Passe la commande en mode "SETTINGS"
    def activerSettings ( ):

        global isSettings
        isSettings = True

        asr.set_property('lm', directory + 'baxter_settings.lm')
        asr.set_property('dict', directory + 'baxter_settings.dic')

    # Passe la commande en mode normal
    def desactiverSettings ( ):

        global isSettings
        isSettings = False
        
        global sleeping
        sleeping = True

        asr.set_property('lm', directory + 'baxter.lm')
        asr.set_property('dict', directory + 'baxter.dic')

        output ( "Going to sleep now.")

    # Menu des réglages
    def settingsMenu ( text ):

        msg_str = [] 

        # Changement de repère d'instrument
        if "INSTRUMENT" in text:

            # On enlève le mot "instrument "
            text = text[11:]
            #print "text is now :" + text 

            if text in numbers_bindings:

                msg_str = CMD_CONF + ' ' + CMD_INSTRU + ' ' + numbers_bindings[ text ]

        elif "CALIBRATE" in text or "TROCAR" in text:

            msg_str = CMD_CONF + ' ' + CMD_TROCART

        if msg_str:
            msg = String ( msg_str )
            intr_pub.publish ( msg )

        desactiverSettings ()

    
    # Recoit les résultats finaux et envoie la commande correspondante
    def final_result( hyp, uttid):

        global sleeping
        global isSettings
        
        # vérifier qu'on est pas en train de "dormir"
        if not sleeping :

            # On veut nous endormir
            if "SLEEP" in hyp:

                sleeping = True

                output ( "Voice command OFF" )

            # Commande de réglage
            elif "SETTINGS" in hyp:

                activerSettings()

            # On teste la commande
            else:

                testerCommandesDeplacement ( hyp )

        # Si c'est une commande pour nous réveiller
        elif ( "WAKE" in hyp and "UP" in hyp ) or "BAXTER" in hyp:

            sleeping = False
            output ( "Voice command ON" )
            baxter_nod()

        # Si c'est un stop
        elif "STOP" in hyp:

            envoyerStop ()

        else:

            output ( ".." )

    

   # Recoit les résultats finaux, on ne regarde que les commandes courtes 
    def asr_result( asr, hyp, uttid):
        if hyp and len(hyp) < 20:

            hyp = hyp.upper()
            
            # Affichage de la commande en CAPITALES
            output ( hyp )

            if isSettings:
                settingsMenu ( hyp )

            else:
                final_result ( hyp, uttid )

            
    # """ Recoit les résultats partiels """
    def asr_partial_result( asr, text, uttid):
        partial_result ( text, uttid )



    # """ Initialisation de GStreamer et des listeners """
    pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
    asr = pipeline.get_by_name('asr')
    asr.connect('partial_result', asr_partial_result)
    asr.connect('result', asr_result)

    # """ Chargement des fichiers de configurations :
    #    Ils sont dans le dossier commande vocale du dossier des scripts,
    #    on crée le chemin absolu correspondant """
    directory = os.path.dirname(os.path.abspath(__file__));
    directory += '/fichiers_commande_vocale/'
    print ( directory )
    asr.set_property('hmm', directory + 'hub4wsj_sc_8kadapt/')
    asr.set_property('lm', directory + 'baxter.lm')
    asr.set_property('dict', directory + 'baxter.dic')

    # """ Lancement de la reconnaissance vocale """
    asr.set_property('configured', True)
    asr.set_property('dsratio', 1)          # WTF is this ??
    pipeline.set_state(gst.STATE_PAUSED)

    # """ Initialisation du noeud ROS """
    rospy.init_node ( 'commande_vocale' )
    sleeping = True

    # Definition du callback pour le signal SIGINT
    signal.signal(signal.SIGINT, signal_handler)

    # Initialisation des communications
    intr_pub = rospy.Publisher ( TOPIC_CMD_TXT, String )    # Envoie d'une commande à l'interpréteur

    # """ Début de l'écoute """
    pipeline.set_state(gst.STATE_PLAYING)

    rospy.spin()


if __name__ == '__main__':
    main()
    

