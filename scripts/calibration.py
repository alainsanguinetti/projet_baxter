#!/usr/bin/python
# -*- coding: utf8 -*-
#!/usr/bin/env python
from __future__ import unicode_literals
import time
import rospy
import baxter_interface
import baxter_external_devices
import sys
import signal
from cgkit.cgtypes import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import numpy as np
import os
from pylab import *
import math
import tf
import numpy as np
from numpy import linalg as Li
import matplotlib.pyplot as plt 
# Nos librairies et fonctions de configuration
from chaines import *
from utils import *
#from projet_rob4.utils import *
#import projet_rob4.instru
limb = ''
Matrice=[]
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# a partir des mouvements du poignet de Baxter, on retrouve le centre du trocart, 200 valeurs extraites au maximum  




def calibration(limb) :
	pub = rospy.Publisher(TOPIC_CTR_TROCART, Point, latch=True)
	r_limb = baxter_interface.Limb('right')
	output('Calibration')
	time.sleep(1)
	output('Debut de la calibration') 
	time.sleep(1)
	output('Calibration en cours ... ')

	#Matrice contient les positions 
	Matrice=[]
	compteur=0
	#M : contient les informations sur la droite representant l'endoscope 
	M=[]
	cond=1
	while(cond):
		M=mesure(M,limb)
		x=MMC(M)
		if x [0][0]!=10:
			Matrice.append([x[0][0],x[1][0],x[2][0]])
			cond=0
		else :
			o=0
			while(o<10):
				print(np.size(M,0))
				np.delete(M,np.size(M,0))
				o=o+1

	#N=Li.norm(x)
	while compteur<100 : 
		print compteur 
		M=mesure(M,limb)
		T=MMC(M)
		if T[0][0]!=10 :
			Matrice.append([T[0][0],T[1][0],T[2][0]])
			# erreur de 5 mm 
			n=np.size(Matrice,0)
			print(Li.norm (np.array(Matrice[n-1][:])-transpose(T))+Li.norm( np.array(Matrice[n-2][:])-transpose(T)) +Li.norm( np.array(Matrice[n-1][:])-transpose(T)  ) ) 
			if compteur>20 and  n>3 and  Li.norm( np.array(Matrice[n-1][:] ) -transpose(T)  ) < 0.005 and Li.norm( np.array(Matrice[n-2][:])- transpose(T) )< 0.005  and Li.norm( np.array(Matrice[n-3][:]) -transpose(T)  )<0.005 : 
				message=Point(T[0][0],T[1][0],(T[2][0]+0.1))
				pub.publish (message)
				output('fin de la calibration')
				return
			else : 
				x=T
				compteur=compteur+1
		else :
			o=0
			while(o<10):
				np.delete(M,np.size(M,0))
				o=o+1
	message=Point(T[0][0],T[1][0],(T[2][0]+0.1))
	pub.publish (message)
	output('fin de la calibration')
	return
	




def mesure(M,limb) : 
	 #ajoute 10 lignes a M : on prend 10 mesure de droite 
	for i in range (0,10) :
		t=time.time() 
		coordonnees = limb.endpoint_pose()['position']
		orientation=limb.endpoint_pose()['orientation']
		message1 = Point ( coordonnees.x, coordonnees.y, coordonnees.z )
		message2 = Quaternion ( orientation.x, orientation.y, orientation.z,orientation.w )
		M.append([coordonnees.x,coordonnees.y,coordonnees.z,orientation.x, orientation.y, orientation.z,orientation.w])
		if 0.1 -( t-time.time() )>0 : 
			time.sleep( 0.1 -( t-time.time() ) ) 
	return M 
	
def MMC(M) : 
	#methode des moindres carrees Ax= B , return x 
	for j in range(0,np.size(M,0)) :
	# mesure l'orientation de l'instrument 	
		angle=tf.transformations.euler_from_quaternion(M[j][3:7])
		M[j][3]= angle[0]
		M[j][4]= angle[1]
		M[j][5]= angle[2]
		#print(M)
	A=np.zeros(shape=(np.size(M,0)*2,3))
	B=np.zeros(shape=(np.size(M,0)*2,1))
	
	for i in range (0,np.size(M,0)) :
	# mesure des coefficients de droites : Matrices A et B 		
		alpha=M[i][3]
		beta=M[i][4]
		gama=M[i][5]
		Rx=[[1,0,0],[0,math.cos(alpha),-math.sin(alpha) ],[0,math.sin(alpha),math.cos(alpha)]]
		Ry=[[math.cos(beta),0,math.sin(beta)],[0,1,0],[-math.sin(beta),0,math.cos(beta)]]
		Rz=[[math.cos(gama),-math.sin(gama),0],[math.sin(gama),math.cos(gama),0],[0,0,1]]

		Rot=np.dot(np.dot(Rz,Ry),Rx)

		x=M[i][0]
		y=M[i][1]
		z=M[i][2]

		Zx=Rot[0][2]
		Zy=Rot[1][2]
		Zz=Rot[2][2]

		A[2*i][1]=Zz
		A[2*i][2]=-Zy
		A[2*i+1][0]=-Zz
		A[2*i+1][2]=Zx

		B[2*i][0]=y*Zz-z*Zy
		B[2*i+1][0]=z*Zx-x*Zz
		
	Tmp=np.dot(np.transpose(A),A)
	if  np.linalg.matrix_rank(Tmp)==Tmp.shape[1] :
		Y=np.linalg.inv(Tmp)
		X= np.dot(Y ,np.transpose(A))
		x = np.dot(X,B)
		return x
	else : 
		x=np.array([ [10],[10],[10] ])
		return x


