#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import time
import datetime

from hsrb_interface import geometry
import hsrb_interface

import math
import csv
from std_srvs.srv import Empty, EmptyRequest
robot = hsrb_interface.Robot()
whole_body = robot.get('whole_body')
tts = robot.get('default_tts')
omni_base = robot.try_get('omni_base')

import tf



from hsr_handing.msg import face_info


class getIm_from_HSR(object):

    def __init__(self):
        rgb = r'/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        depth = '/hsrb/head_rgbd_sensor/depth_registered/image'
        self._bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
	self.sub_count_rgb = 0
	self.sub_count_dep = 0
        # Subscribe color image data from HSR
        rospy.Subscriber(rgb, Image, self.callback)
        rospy.Subscriber(depth, Image, self.callback2)
        #self._image_sub = rospy.Subscriber(topic_name, Image, self.callback)
        # Wait until connection
        rospy.wait_for_message(rgb, Image, timeout=15.0)
        rospy.wait_for_message(depth, Image, timeout=15.0)
    def callback(self, data):
        try:
            self.rgb_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
	    self.sub_count_rgb +=1
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)
    def callback2(self, data):
        try:
            self.depth_image = self._bridge.imgmsg_to_cv2(data, "32FC1")
            self.sub_count_dep +=1
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)


class face_detection():
    def __init__(self):
        self.face_list = []
        self.face_num = 0
        self.face_x = 0
        self.face_y = 0
        self.width = 0 
        self.height = 0
        self.Vx = 0
        self.Vy = 0
        self.detection_time = 0
    def detect(self,input_image):
        
        cascade_path = r'/home/naoyamada/catkin_ws3/src/hsr_handing/script/haarcascades/haarcascade_frontalface_alt2.xml'
        cascade = cv2.CascadeClassifier(cascade_path)
        face_list = cascade.detectMultiScale(input_image,scaleFactor=1.5, minNeighbors=2) 
        self.face_list = np.array(list(face_list))
        #self.detection_time = time.time()
        #顔の数
        self.face_num = len(self.face_list)
        #顔パラメータの取得                
        for (x, y, w, h) in self.face_list:            
            #顔の中心
            self.face_x = x+w/2
            self.face_y = y+h/2
            cv2.rectangle(input_image, (x, y), (x+w, y+h), color=(0, 0, 225), thickness=3) 
            #画像中心までのベクトル定義
            self.height, self.width, channels = input_image.shape[:3] 
            self.Vx = self.width/2 - self.face_x
            self.Vy = self.height/2 - self.face_y

        return input_image

def create_mask_image(channel, th_max, th_min):
        ret, src = cv2.threshold(channel, th_max, 255, cv2.THRESH_TOZERO_INV)
        ret, dst = cv2.threshold(src, th_min, 255, cv2.THRESH_BINARY)
        return dst

def skin_detect(img):
    H_LOW_THRESHOLD = 0
    H_HIGH_THRESHOLD = 15
    S_LOW_THRESHOLD = 50
    S_HIGH_THRESHOLD = 255
    V_LOW_THRESHOLD = 50
    V_HIGH_THRESHOLD = 255
    # BGR -> HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Break down channels
    H, S, V = cv2.split(hsv)
    # Create mask images(H, S, V)
    h_dst = create_mask_image(H, H_HIGH_THRESHOLD, H_LOW_THRESHOLD)
    s_dst = create_mask_image(S, S_HIGH_THRESHOLD, S_LOW_THRESHOLD)
    v_dst = create_mask_image(V, V_HIGH_THRESHOLD, V_LOW_THRESHOLD)
    dst = cv2.bitwise_and(h_dst, s_dst)

    return dst

# Calcurate running_average
_AVERAGE_RANGE = 10
depth_elements=[0]*_AVERAGE_RANGE
center_elements=[0]*_AVERAGE_RANGE
def running_average(elements,row_deta):

    for i in range(_AVERAGE_RANGE-1):
        elements[i] = elements[i+1]
    elements[_AVERAGE_RANGE-1] = row_deta
    return round(sum(elements)/_AVERAGE_RANGE,2)


class parameter():
    def __init__(self):
        self.depth_list = [0]*10

def face_tracking():
    sub_cycle = 100
    pan = 0.0
    tilt = 0.0
    pan_old = 0.0
    tilt_old = 0.0
    yaw = 0.0
    detection_count = 0
    old_rgb = 0
    old_dep = 0
    #保存用
    #basename = 'test1'
    #dir_path = '/home/naoyamada/catkin_ws2/src/pkg1/IMAGE'
    #ext = 'jpeg'
    #base_path = os.path.join(dir_path, basename)
    #save_cycle = 1


    try:
        HSR_image = getIm_from_HSR()
        Face = face_detection()
        param = parameter()
        spin_rate = rospy.Rate(sub_cycle)
        pub = rospy.Publisher('face_info_topic', face_info, queue_size=10)
        Hello = True
        writing = True
	
        while not rospy.is_shutdown():

            while  not rospy.is_shutdown():
                #print(Face.detection_time)
                Human_Face = False
                tracking = False
                detection_image = Face.detect((HSR_image.rgb_image))
                depth_image = HSR_image.depth_image
                pre_rgb = HSR_image.sub_count_rgb
                pre_dep = HSR_image.sub_count_dep
                cv2.imshow("Face Detection", detection_image)
                #cv2.imshow("Depth Image", depth_image)
                
                #フレームの保存
                #if n % save_cycle == 0:    
                    #cv2.imwrite('{}_{}.{}'.format(base_path, n, ext),detection_image )
                #n += 1
                
                if (Face.face_num==1):
                    #検出した顔の範囲をより小さい範囲に限定する
                    w= int(Face.face_list[0][2]*0.6)
                    h= int(Face.face_list[0][3]*0.6)
                    x= int(Face.face_x-w/2)
                    y= int(Face.face_y-h/2)
                    #顔範囲の切り出し,大きさの計算
                    face_cut = detection_image[y:y+h,x:x+w]
                    depth_face_cut = depth_image[y:y+h,x:x+w]
                    face_skin = skin_detect(face_cut)
                    face_h, face_w= face_skin.shape[:2]
                    face_area = face_h*face_w
                    cv2.imshow("Face_skin", face_skin)
                    face_pixel = int(np.average(face_skin))
                    #print(face_pixel)
                cv2.waitKey(3)

                #一人の人間の顔判定
                if (Face.face_num==1)and(face_pixel>40):
                    Human_Face = True
                    detection_count += 1
                    
		#人間の顔であればDepth取得
                if Human_Face==True:
                    cv2.rectangle(face_cut, (x, y), (x+w, y+h), color=(255, 0, 0), thickness=3) 
                    face_depth = round(np.mean(depth_image[y:y+h,x:x+w]),2)
                    center_depth = round(np.mean(depth_image[240,320]),2)
                    #両方nanでなければDepthを発行
                    if np.isnan(face_depth)==False and np.isnan(center_depth)==False:
			#pub.publish(face_depth,center_depth,Face.Vx,Face.Vy,Face.detection_time)
                        pub.publish(face_depth,center_depth,Face.Vx,Face.Vy,0)
                        
		#if Human_Face == False:
                    #pub.publish(0,0,0,0)
		    

		#if (old_rgb != pre_rgb):
                    #print("-------------new frame------------------")
		#顔追従
                if (abs(Face.Vx)>20 or abs(Face.Vy)>20)and(Human_Face==True)and(detection_count>1):
		    tracking=True
                if (abs(Face.Vx)<20 and abs(Face.Vy)<20):
		    tracking = False
                    
                
                if tracking == True:
                    
                    pan_weight = 0.0007
                    tilt_weight = 0.0007             

                    pan = pan_old + pan_weight*Face.Vx
                    tilt = tilt_old + tilt_weight*Face.Vy
                    
                    if abs(pan)<0.01:
                        pan=0
                    if abs(tilt)<0.01 :
                        tilt=0

                    if tilt>0.52:
                        tilt=0.52
                    if pan>1.74:
                        pan = 1.74
                        yaw = 0.005*Face.Vx
                    if pan<-3.83:
                        pan = -3.83
	            whole_body.move_to_joint_positions({'head_pan_joint': pan, 'head_tilt_joint': tilt})
	            pan_old = pan
                    tilt_old = tilt
		    '''
                    try:
			#now = rospy.Time.now()
                        whole_body.move_to_joint_positions({'head_pan_joint': pan, 'head_tilt_joint': tilt})
		        pan_old = pan
                        tilt_old = tilt
		    except:
			rospy.logerr('aa')
       		    '''
                spin_rate.sleep()
        
    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)

    cv2.destroyAllWindows()




if __name__ == '__main__':
    face_tracking()
