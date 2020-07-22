#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry
import math
import csv
import numpy as np
import sys
import time

from hsr_handing.msg import face_info
from hsr_handing.srv import input4handing
from hsr_handing.srv import input4handingResponse
from std_msgs.msg import Bool
# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

class Sub_face_info():
    def __init__(self):
        rospy.Subscriber("face_info_topic", face_info, self.callback)
	self.face = 0
	self.center = 0
        #self.detection_time = 0
    def callback(self, data):
        self.face = data.face_depth
        self.center = data.center_depth
 	#self.detection_time = data.time

def handing(req):
    

        D = 2
        #d = sub_face.center
        #print('{},{}'.format(D,d))
        pan = np.round(whole_body.joint_positions['head_pan_joint'],2)
        tilt = np.round(whole_body.joint_positions['head_tilt_joint'],2)

        #print('{}, {}'.format(pan, tilt))
        #-----------head_pan_link からface への座標変換--------------#
        i = np.array([[1,0,0],
                      [0,math.cos(0),math.sin(0)],
                      [0,-math.sin(0),math.cos(0)]])
        j = np.array([[math.cos(tilt),0,-math.sin(tilt)],
                      [0,1,0],
                      [math.sin(tilt),0,math.cos(tilt)]])
        k = np.array([[math.cos(pan),math.sin(pan),0],
                      [-math.sin(pan),math.cos(pan),0],
                      [0,0,1]])


        pan2tilt = np.array([0.02,0,0])
        tilt2camera = np.dot(j,np.array([-0.0798,0.022,0.2152]))
        pan2camera = pan2tilt + tilt2camera

	base2pan = np.dot(k,np.array([0,0,0.752]))
        base2camera = base2pan + pan2camera

        print('pan2camera = {}'.format(pan2camera))
	print('base2camera = {}'.format(base2camera))
        if tilt>=0.52:
            tilt = math.acos(d*math.cos(0.52)/D)     
        #x = D*math.cos(tilt)*math.cos(pan)
        x = D*math.cos(tilt)
        #y = D*math.cos(tilt)*math.sin(pan)
        y = 0
        z = D*math.sin(tilt)
        camera2face = np.array([x,y,z])
        #print(camera2face)
               
        pan2face = pan2camera + camera2face
        face2hand = np.array([-0.5, 0,-0.5])
                
        #print(pan2face)
        pan2hand = pan2face + face2hand
        Hand_TF = np.round(pan2hand,2)
        #print(Hand_TF)
        handing_point = geometry.pose(Hand_TF[0],Hand_TF[1],Hand_TF[2],ei=1.57*2,ej=-1.57)
        #------------------------------------------------------#
        #rospy.sleep(0.001)
	#手渡し    
        whole_body.move_end_effector_pose(handing_point, ref_frame_id='head_pan_link')
        
        handing_END = True
        

    return input4handingResponse(action_name)
  





if __name__=='__main__':
    handing_END = False
    try:
	server1 = rospy.Service('input_topic4handing', input4handing, handing)
	param_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/param.csv'
        while not rospy.is_shutdown():
            sub_face  = Sub_face_info()
            #pub_flag = rospy.Publisher("flag_topic", Bool, queue_size=10)  
            #pub_flag.publish(handing_END)
            #handing_END = False
            rospy.spin()
            
    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)




