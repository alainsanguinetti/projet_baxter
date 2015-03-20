#!/usr/bin/env python

"""
    Copyright (c) 2013, HumaRobotics
    All rights reserved.

    
    Helper Classes and functions for Baxter
    
"""

import argparse
import sys
import pickle


import roslib; roslib.load_manifest('baxter_pickplace')
import rospy

from post_threading import Post

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import baxter_core_msgs.msg as baxmsg
import std_msgs.msg as stdmsg

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    ListCameras,
    )
from baxter_core_msgs.msg import (    
    EndEffectorProperties,
    )
import baxter_interface

import baxter_dataflow as dataflow
from imagetextdisplayer import ImageTextDisplayer
from imagecomposer import ImageComposer,ImageComposerElement
import cv
import cv_bridge

import tf
import numpy
import math
from copy import deepcopy

import PIL 

class BaxterException(Exception): pass


class Gripper(baxter_interface.Gripper):
    """ Extends baxter gripper interface with grip and release  commands, replaces open/close """
    def __init__(self,side):
        if not side in ["left","right"]:
            raise BaxterException,"Error non existing side: %s, please provide left or right"%side        
        baxter_interface.Gripper.__init__(self,side)
        self.side=side
        self.post=Post(self)
        self.epsilon = 0.5
        self.open_position = 100
        self.close_position = 0
        self.opened = True
        self.outside_grip = True
        
    def open(self,blocking = True):
        baxter_interface.Gripper.open(self,blocking)
        self.opened = True
        
    def close(self,blocking = True):
        baxter_interface.Gripper.close(self,blocking)
        self.opened = False
        
    def calibrate_advanced(self):
        self.open()
        self.open_position = self.position()
        self.close()
        self.close_position = self.position()
        self.open(False)
        
    def gripped(self):
        print "max open %f, max close %f, cur pos %f"%(self.open_position,self.close_position,self.position())
        if self.type() == 'electric':
            if self.outside_grip:
                return [False, True][self.position() > self.close_position + self.epsilon]
            else:
                return [False, True][self.position() < self.open_position - self.epsilon]
        if self.type() == 'suction':
            return self.gripping()
        
    def grip(self,blocking = True):
        [self.open(blocking),self.close(blocking)][self.outside_grip]
                   
    def release(self):
        [self.close(blocking),self.open(blocking)][self.outside_grip]
    
class BaxterDisplay():
    def __init__(self):
        self._display_pub= rospy.Publisher('/robot/xdisplay',       Image       )
    
    def setImage(self,data):
#         print "data",data
#         print "type",type(data)
        while not rospy.is_shutdown() and self._display_pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        if type(data) == str:
            img = cv.LoadImage(data)        
            data = cv_bridge.CvBridge().cv_to_imgmsg(img)
        self._display_pub.publish(data)

class BaxterTextDisplayer:
    def __init__(self,datapath,display):
        self.post=Post(self)      
        self.text=""
        self.tmpfile="/tmp/display.jpg"
        self.fontfile=datapath+"font/UbuntuMono-R.ttf"
        #print "fontfile",self.fontfile
        self.displayer=ImageTextDisplayer(self.fontfile)
        self.display = display
        self.autorefresh=False
        
    def log(self,text,refresh=False):
        #~ print text
        self.text=self.text+text
        if refresh:
          self.refresh()

    def clear(self,refresh=False):
        self.text=""
        if refresh:
            self.refresh()
    
    def write(self,data):
        self.log(str(data))
        
    def refresh(self):
        self.displayer.drawText(self.tmpfile,self.text) 
        self.display.setImage(self.tmpfile)

    def autoRefresh(self,period=1):
        if self.autorefresh:
            return
        self.autorefresh=True
        while not rospy.is_shutdown() and self.autorefresh:
            self.refresh()
            rospy.sleep(period)
        self.refresh()
    
    def stopAutoRefresh(self):
        self.autorefresh=False
        

    
    
class DigitalIO(baxter_interface.DigitalIO):
    """ Extends DigitalIO with waitForPressed """
    
    def __init__(self,id,led=None):
        self.led = led
        self.post=Post(self)
        self.enableState = [False, True][id.find("shoulder") == -1]
        baxter_interface.DigitalIO.__init__(self,id)
        if id.startswith("left"):
            self.lid = "left_itb_light_inner"
        elif id.startswith("right"):
            self.lid = "right_itb_light_inner"
        else:
            rospy.loginfo("No valid button selected")
    
    def activateLed(self):
        if self.led!=None:
            self.led.post.blink(self.lid, 0.5, 4)
    
    def waitForClick(self):                
        while not rospy.is_shutdown() and self.state==0: # Wait pressed
            rospy.sleep(0.1)
        self.activateLed()
        while not rospy.is_shutdown() and self.state!=0: # Wait released
            rospy.sleep(0.1)
            
    def callbackOnPressed(self,callback):
        self.post._callbackOnPressed(callback)

    def _callbackOnPressed(self,callback):
        last=(not self.enableState)
        while not rospy.is_shutdown(): # Wait pressed
            if last==(not self.enableState) and self.state==self.enableState:
                self.activateLed()
                callback()
            last=self.state
            rospy.sleep(0.1)
        

class Navigator(baxter_interface.Navigator):
    """ Extends Navigator with waitForPressed  and pressed callbacks
    
    Navigator
    """
    
    def __init__(self,id,led=None):
        self.led = led
        self.post=Post(self)
        baxter_interface.Navigator.__init__(self,id)
        self.id = id + "_itb_light_inner"
        
    def activateLed(self):
        if self.led!=None:
            self.led.post.blink(self.id, 0.5, 4)
    
    def waitForClickOk(self):
        self._waitForClick(lambda: self.button0)

    def waitForClickCancel(self):
        self._waitForClick(lambda: self.button1)

    def waitForClickShow(self):
        self._waitForClick(lambda: self.button2)
        
        
    def _waitForClick(self,statefunc):
        while not rospy.is_shutdown() and statefunc()==False: # Wait pressed
            rospy.sleep(0.1)
        self.activateLed()
        while not rospy.is_shutdown() and statefunc()==True: # Wait released
            rospy.sleep(0.1)
        
            
    def callbackOnPressedOk(self,callback):
        self._callbackOnPressed(lambda: self.button0,callback)

    def callbackOnPressedCancel(self,callback):
        self._callbackOnPressed(lambda: self.button1,callback)

    def callbackOnPressedShow(self,callback):
        self._callbackOnPressed(lambda: self.button2,callback)
        
    def callbackWheel(self,callback):
        self.post.__callbackWheel(lambda: self.wheel,callback)
        
    def __callbackWheel(self,statefunc, callback):
        last = statefunc()
        while not rospy.is_shutdown():
            newstate = statefunc()
            if last!=newstate:
                #print newstate
                self.newstate = newstate
                #import inspect
                #print inspect.getsource(callback)
                self.activateLed()
                callback()#TODO pass parameter to callback
                last = newstate
            rospy.sleep(0.1)

    def _callbackOnPressed(self,statefunc,callback):
        self.post.__callbackOnPressed(statefunc,callback)

    def __callbackOnPressed(self,statefunc,callback):
        last=statefunc()
        while not rospy.is_shutdown(): # Wait pressed
            #~ print last
            newstate=statefunc()
            if last==False and newstate==True:
                self.activateLed()
                callback()
            last=newstate
            rospy.sleep(0.1)
        
            

class Head(baxter_interface.Head):
    def __init__(self):
        baxter_interface.Head.__init__(self)
        self.post=Post(self)

    def command_deny(self, timeout=5.0):
        """
        Command the shake head  "No"

        @param timeout (float)  - Seconds to wait for the head to shake.
                                  If 0, just command once and return.  [0]
        """
        self.set_pan(-0.3)
        self.set_pan(0.3)
        self.set_pan(0)
        
        
class Limb(baxter_interface.Limb):
    def __init__(self,side,ik=True):
        if not side in ["left","right"]:
            raise BaxterException,"Error non existing side: %s, please provide left or right"%side
        baxter_interface.Limb.__init__(self,side)
        self.side=side
        self.ik=ik
        self.post=Post(self)

        if self.ik:            
            #print "Waiting for inverse kinematics service..."
            self.ns = "/ExternalTools/%s/PositionKinematicsNode/IKService"%self.side    
            rospy.wait_for_service(self.ns)
            self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
            #print "Waiting for inverse kinematics service DONE"
        else:
            print "Skipping inverse kinematics service loading"

        #~ self.set_joint_position_mode()


    def buildCartesianPose(self,x,y,z):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        if self.side=="right":
            o=Quaternion(x=0.37953910891620285, y=0.9239036874838116, z=-0.01182849389081732, w=0.047033262582820284)
        else:
            o=Quaternion(x=0.37953910891620285, y=0.9239036874838116, z=-0.01182849389081732, w=0.047033262582820284) # TODO
        goal=PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(x=x,y=y,z=z),
                    orientation=o
                )
            )
        return goal
        
    def getRelativeCartesianPose(self,pose,dx,dy,dz):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        p=pose["position"]
        o=pose["orientation"]
        goal=PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(x=p.x+dx,y=p.y+dy,z=p.z+dz),
                    orientation=o
                )
            )
        return goal

    def goToCartesianXYZ(self,x,y,z):
        goal=self.buildPose(x,y,z)
        self.goToCartesianPose(goal)
        
    def goToCartesianPose(self,goal):
        angles=self.getAnglesFromCartesianPose(goal)
        if angles:
            self.move_to_joint_positions(limb_joints)

    def getAnglesFromCartesianPose(self,goal):
        if not self.ik:
            raise BaxterException,"Inverse Kinematics service was not loaded"
        goalstr="%f,%f,%f"%(goal.pose.position.x,goal.pose.position.y,goal.pose.position.z)
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(goal)
        try:
            resp = self.iksvc(ikreq)
        except rospy.ServiceException,e :
            rospy.loginfo("Service call failed: %s" % (e,))
            return None
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found for %s"%goalstr)
            # Format solution into Limb API-compatible dictionary
            print resp.joints[0]
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else:
            print("FAILURE - No Valid Joint Solution Found for %s"%goalstr)
            return None
        
    def saveCurrentAnglesToFile(self,filename):
        self.saveAnglesToFile(filename,self.joint_angles())

    def saveAnglesToFile(self,filename,angles):
        pickle.dump( angles, open( filename, "wb" ) )     

    def loadAnglesFromFile(self,filename):
        return pickle.load(open(filename,"rb"))

    def goToAnglesFromFile(self,filename):
        angles=self.loadAnglesFromFile(filename)        
        self.move_to_joint_positions(angles)

    def move_to_joint_positions_pid(self,angles,kp=2,threshold=0.02,timeout=15):
        """
        @param positions dict({str:float})   - dictionary of joint_name:angle
        @param kp - P coefficient of PID [2]
        @param timeout    - seconds to wait for move to finish [15]

        Commands the limb to the provided positions.  Waits until the reported
        joint state matches that specified.
        """

        #~ self.set_joint_velocity_mode()

        try:
            dataflow.wait_for(
                lambda goal=angles,threshold=threshold: self._move_to_joint_positions_pid_finished(goal,threshold),
                timeout=timeout,
                rate=100,
                body=lambda goal=angles,kp=kp: self._move_to_joint_positions_pid_do(goal,kp)
                )
        finally:
            self.exit_control_mode()

    def _move_to_joint_positions_pid_do(self,goal,kp):
        current=self.joint_angles()
        for k in goal.keys():
            g=goal[k]
            c=current[k]
            s=(g-c)*kp
            current[k]=s
        self.set_joint_velocities(current)

    def _move_to_joint_positions_pid_finished(self,goal,threshold):
        current=self.joint_angles()
        max=0
        for k in goal.keys():
            g=goal[k]
            c=current[k]
            d=abs(g-c)
            if d>max: max=d
        #~ print "max=%f"%max
        if max<threshold:
            return True
        else:
            return False
        
class Led():
    #    
    def __init__(self):
        self.post = Post(self) 
        self.led_names = ['left_itb_light_outer','left_itb_light_inner', 
                          'right_itb_light_outer', 'right_itb_light_inner',
                          'torso_left_itb_light_outer','torso_left_itb_light_inner',
                          'torso_right_itb_light_outer','torso_right_itb_light_inner']
        self.led_handle = {}
        self.led_state = {}
        for led in self.led_names:
            self.led_handle[led] = baxter_interface.DigitalIO(led)
            self.led_state[led] = 0
    
    def enable(self,name):
        self.led_handle[name].set_output(True)
        self.led_state[name] = 1
    
    def disable(self,name):   
        self.led_handle[name].set_output(False)
        self.led_state[name] = 0
        pass
    
    def disableAll(self):
        for led in self.led_names:
            self.disable(led)
        
    
    def blink(self,name, timeout=0, frequency=2): #timeout <= 0 blinks endlessly
        end = rospy.Time.now()+ rospy.Duration(timeout)
        self.led_state[name] = 1
        while not rospy.is_shutdown():
            start = rospy.Time.now()
            if (start > end and timeout > 0) or self.led_state[name] == 0:
                self.led_handle[name].set_output(False)
                break
            self.led_handle[name].set_output(True)
            rospy.Rate(frequency*2).sleep()
            self.led_handle[name].set_output(False)
            rospy.Rate(frequency*2).sleep()
            
    def blinkAllOuter(self,timeout=0,frequency=2):
        for led in xrange(0,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)
        
    def blinkAllInner(self,timeout=0,frequency=2):
        for led in xrange(1,len(self.led_names),2):
            self.post.blink(self.led_names[led],timeout,frequency)
        
 
class Tee:
    def __init__(self,t1,t2):
        self.t1=t1
        self.t2=t2
        
    def write(self,data):
        self.t1.write(data)
        self.t2.write(data)       
    
class BaxterFrame():
    def __init__(self,arm):
        self.listener = tf.TransformListener()
        self.post = Post(self)
        self.state = 0
        self.arm = arm
        self.shaft = {}
        self.top = {}
        self.coords= [{},{},{}]
        self.box_index = 0
        self.num_points = 3
        self.frames = {}
        
    def getPose(self,target_frame,source_frame):
        try:
            latest = rospy.Time(0)
            self.listener.waitForTransform(target_frame,source_frame,latest,rospy.Duration(4.0))
            (trans,rot) = self.listener.lookupTransform(target_frame,source_frame,latest)
            return list(trans)+list(rot)
        except Exception,e:
            print e
            return None
            
    def broadcastNewFrame(self,target_frame, pose,rate=50): 
        #print pose
        tb = tf.TransformBroadcaster()
        self.frames[pose.header.frame_id] = True
        while not rospy.is_shutdown() and self.frames[pose.header.frame_id] == True:
            tb.sendTransform((pose.pose.position),
                (pose.pose.orientation),rospy.Time.now(), pose.header.frame_id, target_frame)
            rospy.Rate(rate).sleep()
            
    def stopFrameBroadcast(self,frame_id):
        try:
            self.frames[frame_id] = False
        except Exception,e:
            print "could not stop frame broadcast",e
        
    def listFrames(self):
        for frame in self.frames.keys():
            print frame
        return self.frames

    def quaternion_from_euler(self, roll,yaw,pitch,axes='sxyz'):
        return tf.transformations.quaternion_from_euler(roll,yaw,pitch,axes)
        
    def euler_from_quaternion(self,quaternion,axes='sxyz'):
        return tf.transformations.euler_from_quaternion(quaternion,axes)
    
    def addPoint(self,side,step):
        if self.state < self.num_points  and self.state >=0:
            self.coords[step] = self.arm[side].endpoint_pose()
            print "saved point ", step, ":", self.coords[step]
            return self.coords[step]
            
    def computeTransformation(self):
        for coords in self.coords:
            if coords == {}:
                print "too little points have been taken. try again"
                print "use the back button to go back"
                return
        if self.state== self.num_points : #compute pose offset
            print "compute pose"
            try:
                p = []
                for coords in self.coords:
                    p.append(self.vector(list(coords['position'])))
            except:
                print "insufficient points, try again"
                print "switched back to mode 0"
                self.state = 0
                return
            pose = PoseStamped()
            pose.header = Header(stamp=rospy.Time.now(),frame_id="box"+str(self.box_index))
            self.box_index+=1
            
            #yaw = self.computeOffsetAngles(p,2)
            yaw = self.computeYaw(p)
            #yaw = 0
            pitch = self.computePitch(p)
            #pitch = 0
            roll = 0

            pose.pose.position = p[0]
            pose.pose.orientation = self.quaternion_from_euler(roll, pitch, yaw)
            
            #self.post.setPose("base", pose)
            self.state = 0
            print "Transformation computed. Switched back to mode 0"
            return pose    
        else: 
            rospy.loginfo("invalid state in baxter frame. state: %d",self.state)
        
    def computeYaw(self,points):
        p = deepcopy(points)
        p[1][2] = p[0][2]
        
        v_12 = p[1]-p[0]
        p_yaw = deepcopy(p[0])
        p_yaw[0]+=10
        v_yaw = p_yaw - p[0]
        #print "v_12",v_12
        #print "v_yaw",v_yaw
        angle = self.computeAngle(v_12, v_yaw)
        if v_12[1] < 0:
            angle = -angle
        print "angle in deg ",self.rad2deg(angle)
        return angle
    
    def computePitch(self,points):
        p = deepcopy(points)
        v_12 = p[1]-p[0]
        p1z0 = p[1]
        p1z0[2] = p[0][2]
        v_pitch = p1z0 - p[0]
        #print "v_12",v_12
        #print "v_pitch",v_pitch
        angle = self.computeAngle(v_12, v_pitch)
        if v_12[2] < 0:
            angle = -angle
        print "angle in deg ",self.rad2deg(angle)
        return angle
    

    def computeOffsetAngles(self,points,id):#roll = 0, pitch = 1, yaw = 2
        p = deepcopy(points)
        p[1][id] = p[0][id] 
        #print "p0",p[0]
        #print "p1",p[1]
        v_12 = p[1]-p[0]
        p_id = deepcopy(p[0])
        p_id[0]+=10
        v_id = p_id - p[0]
        angle = self.computeAngle(v_12, v_id)

        print "angle in deg ",self.rad2deg(angle)
        return angle
    
    def computeVector(self,v1,v2):
        return v2-v1
    
    def computeLength(self,v):
        return numpy.sqrt(v.dot(v))
    
    def rad2deg(self,rad):
        return rad*180/math.pi
    
    def deg2rad(self,deg):
        return deg/180*math.pi
    
    def computeAngle(self,v1,v2):
#         print "v1comp",v1
#         print "v2comp",v2
        top = (v1.dot(v2))
#         print "top",str(top)
#         print "len1",self.computeLength(v1)
#         print "len2",self.computeLength(v2)
        bottom = self.computeLength(v1)*self.computeLength(v2)
#         print "bottom", bottom 
        if math.fabs(bottom)<0.01:
            rospy.loginfo("Error: Point are to close to each other, increase the distance between the points")
            return 0
#         print "no error"
        return numpy.arccos(top/bottom)
    
    def vector(self,coords):
        return numpy.array(coords) 

class BaxterCamera:
    def __init__(self,display, textDisplay):
        self.post = Post(self)
        self.textDisplay = textDisplay
        self.display = display
        self.subscribers = {}
        self.cameras = {}
        self.camera_list = None
        self.getAllCameras()
        self.closeAllCameras()
        
    def getAllCameras(self):
        try:
            srv = "/cameras/list"
            rospy.wait_for_service(srv, 5.0)
            camera_list_srv = rospy.ServiceProxy(srv, ListCameras)
            self.camera_list = camera_list_srv().cameras            
        except Exception, e:
            print e
        for camera_name in self.camera_list:
            try:
                self.cameras[camera_name] = baxter_interface.CameraController(camera_name)            
            except:
                pass
        
    def closeAllCameras(self):
        for camera_name in self.subscribers.keys():
            self.subscribers[camera_name].unregister()
        for camera_name in self.cameras.keys():
            self.cameras[camera_name].close()
            
    def listCameras(self):
        return self.camera_list
    
    def startCamera(self,camera_name):
        if camera_name in self.cameras.keys():
            camera = self.cameras[camera_name]
            camera.resolution = (960, 600)
            try:
                camera.open()
            except Exception,e:
                print "could not open the camera",e
                return
            if self.textDisplay != None:
                self.textDisplay.autorefresh = False
            rospy.sleep(1)
            self.subscribers[camera_name] = rospy.Subscriber('/cameras/' + camera_name + "/image", Image,self.republish,None,1)
        else:
            print "camera not found"
    
        
    def republish(self,msg):
        self.display.setImage(msg)
        
    def closeCamera(self,camera_name):
        if camera_name in self.subscribers.keys():
            self.subscribers[camera_name].unregister()
            try:
                self.cameras[camera_name].close()
            except Exception,e:
                print e
            del self.subscribers[camera_name]
            self.textDisplay.post.autoRefresh(1)
            #print "camera closed, autorefresh enabled"


class BaxterMenu:    
    def __init__(self,display,textDisplay,datapath):
        self.entries = {}
        self.display = display
        self.textDisplay = textDisplay
        self.img = None
        self.tmp_menu_path = "/tmp/menu.jpg"
        self.display_offset = 150
        self.display_width=1024
        self.display_height=600
        self.char_width=16
        self.char_height=32
        self.text_width=self.display_width/self.char_width
        self.text_height=self.display_height/self.char_height
        
        self.font = PIL.ImageFont.truetype(datapath+"font/UbuntuMono-R.ttf", self.char_height)
        self.font_title = PIL.ImageFont.truetype(datapath+"font/UbuntuMono-R.ttf", 128)
        self.line_spacing=self.char_height
        self.title = ""
        self.cols = 0
        self.rows = 0
        self.selection = {}
        self.hidden = True
        
    def loadEntries(self,title,entries):
        self.entries = entries
        self.title = title
        [rows, cols] = self.computeDimensions(len(self.entries))
        self.cols = cols
        self.rows = rows
        self.x_spacing = self.display_width/self.cols
        self.y_spacing = (self.display_height-self.display_offset)/self.rows
        
    def computeDimensions(self,num_entries):
        tmp = math.sqrt(num_entries)
        tmp2 = int(tmp)
        if tmp==tmp2:
            return [tmp2,tmp2]
        else:
            return [[tmp2, tmp2+1][num_entries>tmp2*tmp2+tmp2],tmp2+1]
        
    def show(self):
        if self.textDisplay != None and self.textDisplay.autorefresh == True:
            self.textDisplay.autorefresh = False
            rospy.sleep(1.5)
        if self.img != None:       
            self.saveImage()
            self.display.setImage(self.tmp_menu_path)
            self.hidden = False
            
    def hide(self):
        if self.textDisplay != None and self.textDisplay.autorefresh == False:
            self.textDisplay.post.autoRefresh(1)
        self.hidden = True
    
    def computeRectangle(self,entry):
        [x,y] = self.selection[entry]
        p1x = x - self.x_spacing/2 + self.x_spacing/10
        p1y = y - self.y_spacing/2 + self.y_spacing/10
        p2x = x + self.x_spacing/2 - self.x_spacing/10
        p2y = y + self.y_spacing/2 - self.y_spacing/10
        return ((p1x,p1y),(p2x,p2y))
        
    def addTitle(self):
        x_pos = self.x_spacing/10
        y_pos = self.display_offset/2
        self.draw.text((x_pos, y_pos),self.title,(255,255,255),font=self.font)
        
    def drawMenuEntries(self):        
        self.img = PIL.Image.new("RGB", (self.display_width,self.display_height), "#313133")
        self.draw = PIL.ImageDraw.Draw(self.img)
        
        x_pos = self.x_spacing/2
        y_pos = self.y_spacing/2+self.display_offset
        
        for entry in self.entries:
            chars = len(entry)
            self.selection[entry] = [x_pos,y_pos] 
            self.drawBackground(entry)
            self.draw.text((x_pos-chars*self.char_width/2, y_pos),entry,(0,0,0),font=self.font)
            x_pos +=self.x_spacing
            if x_pos > self.display_width:
                x_pos = self.x_spacing/2
                y_pos +=self.y_spacing
            
        return self.draw

    def drawBackground(self,entry):
        self.draw.rectangle(self.computeRectangle(entry), fill= '#c45221')
        
    def resetSelection(self):
        self.drawMenuEntries()
        self.addTitle()
        
    
    def select(self,entry):
        self.resetSelection()
        coords = self.computeRectangle(entry)
        self.draw.rectangle(coords, outline="#00ff00")
        self.draw.rectangle(((coords[0][0]-2,coords[0][1]-2),(coords[1][0]+2,coords[1][1]+2)), outline="#00ff00")
        self.draw.rectangle(((coords[0][0]-1,coords[0][1]-1),(coords[1][0]+1,coords[1][1]+1)), outline="#00ff00")
        self.show()
        
    def saveImage(self):
        self.img.save(self.tmp_menu_path)
        
        

    
class BaxterRobot:
    def __init__(self):
        #rospy.loginfo("Baxter Robot of Baxter Tasker loaded")
        self.post=Post(self)
        self.gripper={}
        self.arm={}        
        self.dio={}
        self.navigator={}
        self.led=None
        self.camera=None
        self.menu=None
        self.enabler=baxter_interface.RobotEnable()
        self.head=Head()
        self.display=BaxterDisplay()
        self.textDisplay=None
        self.datapath=roslib.packages.get_pkg_dir('baxter_pickplace') + "/data/"
        self.motor_state = False

    def enable(self):
        if self.motor_state:
            print "motors seem to be already enabled"
            return
        self.motor_state = True
        self.enabler.enable()

    def disable(self):
        if not self.motor_state:
            print "motors seem to be already disabled"
            return
        self.motor_state = False
        self.enabler.disable()
             
    def loadAll(self):

#         self.loadTextDisplay(True)
        self.loadLeds()
        self.loadDigitalIO()    
        self.loadNavigator()
        self.loadGripper("right")    
        self.loadGripper("left")    
        self.loadArm("right")    
        self.loadArm("left")   
#         self.loadFrame()
#         self.loadCamera()
#         self.loadMenu()
        rospy.sleep(1)
        print "All components are loaded."
    
    def loadMenu(self):
        if self.textDisplay == None:
            self.loadTextDisplay()
        self.menu = BaxterMenu(self.display, self.textDisplay,self.datapath)
        #DONT FORGET TO LOAD MENU ENTRYS 
    
    def loadCamera(self):
        self.camera = BaxterCamera(self.display,self.textDisplay)
    
    def loadFrame(self):
        if self.arm=={}:
            self.loadArm("right")    
            self.loadArm("left")   
        self.frame = BaxterFrame(self.arm)        
        
       
    def loadGripper(self,side):        
        self.gripper[side]=Gripper(side)
        gripper_id = rospy.wait_for_message('/robot/end_effector/'+side+'_gripper/properties',EndEffectorProperties,5.0)
        gripper_name = gripper_id.product
        if gripper_name == 'Electric Parallel Gripper':
            self.gripper[side].reboot()
            self.gripper[side].calibrate()
            
            #print gripper_name,"loaded"
        elif gripper_name == 'Suction Cup Gripper':
            #print gripper_name,"loaded"
            pass
        else:
            print "Unknown Gripper with name",gripper_name,"loaded"
            pass
        

    def loadArm(self,side):        
        self.arm[side]=Limb(side)
        rospy.sleep(0.5) # wait until subscribers are active
        
    def loadLeds(self):
        self.led = Led()

    def loadDigitalIO(self):
        l=["right_lower_button","right_upper_button","right_lower_cuff","right_shoulder_button",
           "left_lower_button" ,"left_upper_button" ,"left_lower_cuff" ,"left_shoulder_button"]
        if self.led==None:
            self.loadLeds()
        for id in l:
            self.dio[id]=DigitalIO(id,self.led)
            
    def loadNavigator(self):
        l=["right","left","torso_right","torso_left"]
        if self.led==None:
            self.loadLeds()
        for id in l:
            self.navigator[id]=Navigator(id,self.led)
            
    def yes(self):
        self.head.command_nod()

    def no(self):
        self.head.command_deny()
    
    def loadTextDisplay(self,autostdout=False):        
        self.textDisplay=BaxterTextDisplayer(self.datapath,self.display)
        if autostdout:
            import sys
            sys.stdout=Tee(self.textDisplay,sys.stdout)
            sys.stderr=Tee(self.textDisplay,sys.stderr)
            self.textDisplay.post.autoRefresh(1)
    

    
if __name__ == '__main__':    
    rospy.init_node("baxter_helper")
    baxter=BaxterRobot()
    baxter.loadArm()
    #baxter.enable()
    #baxter.loadDigitalIO()    
    #baxter.loadGripper("left")    
    #baxter.setImage(baxter.datapath+"starting.png")
    #baxter.loadTextDisplay(True)
#     baxter.loadLeds()
#     print "before post"
#     lled = baxter.led.post.blink("left_itb_light_inner")
#     print "after post"
#     rospy.sleep(2)
#     print "before kill"
#     print dir(lled)
#     print "after kill"
#     
#     baxter.loadFrame()
#     rospy.sleep(3)
#     baxter.frame.addPoint("left")
#     baxter.frame.state+=1
#     baxter.frame.addPoint("left")
#     baxter.frame.state+=1
#     pose = baxter.frame.computeTransformation()
#     print pose.pose.position.tolist()
#     
#     pose.pose.position = numpy.array([1, 2, 3])
#     print pose.pose.position
#     baxter.frame.broadcastNewFrame("base", pose)
   # baxter.loadCamera()
   #print baxter.camera.listCameras()
   # cameraToDisplay = baxter.camera.listCameras()[2]
   # baxter.camera.startCamera(cameraToDisplay)#"left_hand_camera")#head_camera")
    #baxter.frame.computeTransformation("right")
    #baxter.display.log("ngdsfggggggggggggggggggggggggggggggs",True)
    #baxter.gripper["left"].calibrate()
    #baxter.gripper["left"].open()
    ##~ rospy.sleep(2)
    #baxter.gripper["left"].close()
    #~ rospy.sleep(2)
    #baxter.gripper["left"].open()
    #~ rospy.sleep(2)
    #baxter.loadArm("right")
    #btn="right_lower_button"
    #print baxter.dio[btn]
    #print "%s state: %d"%(btn,baxter.dio[btn].state)
    #print "Waiting for right_lower_button click"
    #baxter.dio[btn].waitForClick()
    #baxter.disable()
    #rospy.sleep(1)
    rospy.spin()
        
