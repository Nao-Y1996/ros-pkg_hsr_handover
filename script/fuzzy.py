#!/usr/bin/env python
# -*- coding: utf-8 -*-
import setting
import os
import csv
import numpy as np
import rospy
from matplotlib import pyplot as plt
import cv2
import time
from hsr_handing.srv import fuzzy
from hsr_handing.srv import fuzzyResponse
from hsr_handing.msg import pose_info
import math

class Sub_Pose_estimation():
    def __init__(self):
        rospy.Subscriber("pose_topic", pose_info, self.callback)
	self.pose = 3
    def callback(self, data):
        self.pose = data.pose_identification_number

class parameter():
    def __init__(self):       
        
	self.old_ek = -0.0
        self.old_x = -0.5
        self.old_y = 0
        self.oldFX = 0
	self.oldFY = 0
	self.oldfy = 0

class calculate_force():
    def __init__(self):
        self.f = 0
        self.FX = 0
        self.FY = 0
        
    def get_force(self,fx,fy,ek):
        
        if not (fx==0 and fy==0):
            self.f = math.sqrt(math.pow(fx,2)+math.pow(fy,2))
            theta0 = math.degrees(math.acos(abs(fx)/self.f))
        if fx==0 and fy==0:
            theta = 0
            self.f = 0
        if fx>0 and fy>0:#第一象限
            theta = theta0
        if fx<0 and fy>0:#第二象限
            theta = 180-theta0 
        if fx<0 and fy<0:#第三象限
            theta = theta0+180
        if fx>0 and fy<0:#第四象限
            theta = 360-theta0
        if fx>0 and fy==0:#x軸正
            theta = 0
        if fx<0 and fy==0:#x軸負 
            theta = 180
        if fx==0 and fy>0:#y軸正 
            theta = 90
        if fx==0 and fy<0:#y軸負
            theta = 270
        print('f = {}'.format(round(self.f,3)))
        print('theta = {}'.format(round(theta,3)))

        alpha = math.radians(theta) + ek
        self.FX = self.f * math.cos(alpha)
        self.FY = self.f * math.sin(alpha)
        F_list = [self.FX,self.FY]
        return F_list

def triangle_fuzzy(a,c,b,F):#a<c<bとする
    if a<F and F<=c: #(a,0),(c,1)の式
        return 1/(c-a)*(F-a)
    if c<F and F<b:  #(c,1),(b,0)の式
        return 1/(c-b)*(F-b)
    else:
        return 0

def l_edge_fuzzy(c,b,F):#c<bとする
    if F<=c: 
        return 1
    if c<F and F<b:  #(c,1),(b,0)の式
        return 1/(c-b)*(F-b)
    else:
        return 0

def r_edge_fuzzy(a,c,F):#c<bとする
    if a<F and F<=c: #(a,0),(c,1)の式
        return 1/(c-a)*(F-a)
    if c<F:
        return 1                               
    else:
        return 0



class Fuzzy_inference():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.next_ek = 0


    def fuzzy4position(self, x,old_x):
        
        if old_x==0:
	    old_x = x
        dx = x - old_x

        w1 = l_edge_fuzzy(-1.5,0.0,x) * l_edge_fuzzy(-0.7,0.0,dx)
        w2 = l_edge_fuzzy(-1.5,0.0,x) * triangle_fuzzy(-0.2,0.0,0.2,dx)
        w3 = l_edge_fuzzy(-1.5,0.0,x) * r_edge_fuzzy(0.0,0.7,x)

        w4 = triangle_fuzzy(-0.5,0.0,0.5,x) * l_edge_fuzzy(-0.7,0.0,dx)
        w5 = triangle_fuzzy(-0.5,0.0,0.5,x) * triangle_fuzzy(-0.2,0.0,0.2,dx)
        w6 = triangle_fuzzy(-0.5,0.0,0.5,x) * r_edge_fuzzy(0.0,0.7,dx)

        w7 = r_edge_fuzzy(0.0,1.5,x) * l_edge_fuzzy(-0.7,0.0,dx)
	w8 = r_edge_fuzzy(0.0,1.5,x) * triangle_fuzzy(-0.2,0.0,0.2,dx)
	w9 = r_edge_fuzzy(0.0,1.5,x) * r_edge_fuzzy(0.0,0.7,dx)

        w_vec = np.array([w1,w2,w3,w4,w5,w6,w7,w8,w9])
        #print(w_vec)
        then_vec = np.array([ -0.15, -0.1, -0.05, 0.0,0.0,0.0, 0.05, 0.1, 0.15])
        return np.dot(w_vec, then_vec)/np.sum(w_vec)
   
    def fuzzy4angle(self, x,dx):

        w1 = l_edge_fuzzy(-4.0,-3.0,x) * l_edge_fuzzy(-0.7,0.0,dx)
        w2 = l_edge_fuzzy(-4.0,-3.0,x) * triangle_fuzzy(-0.2,0.0,-0.2,dx)
        w3 = l_edge_fuzzy(-4.0,-3.0,x) * r_edge_fuzzy(-0.7,0.0,dx)

        w4 = r_edge_fuzzy(3.0,4.0,x) * l_edge_fuzzy(-0.7,0.0,dx)
	w5 = r_edge_fuzzy(3.0,4.0,x) * triangle_fuzzy(-0.2,0.0,-0.2,dx)
	w6 = r_edge_fuzzy(3.0,4.0,x) * r_edge_fuzzy(0.0,0.7,dx)

        w_vec = np.array([w1,w2,w3,w4,w5,w6])
        #print(w_vec)
        then_vec = np.array([-0.78, -0.52, -0.26, 0.26, 0.52, 0.78])
        return np.dot(w_vec, then_vec)/np.sum(w_vec)



def callback(req):
    pose_num = sub_pose.pose
    if pose_num == 0.0:
        param_path = ex_result_path + '/handing_param/sit.csv'
	ForceXY_path = ex_result_path + '/Force_XY/sit.csv'
    if pose_num == 1.0:
        param_path = ex_result_path + '/handing_param/stand.csv'
	ForceXY_path = ex_result_path + '/Force_XY/stand.csv'
    if pose_num == 2.0:
        param_path = ex_result_path + '/handing_param/cross.csv'
	ForceXY_path = ex_result_path + '/Force_XY/cross.csv'
    if pose_num == 3.0:
        param_path = ex_result_path + '/handing_param/unknown.csv'
	ForceXY_path = ex_result_path + '/Force_XY/unknown.csv'
	    
    fuzzy = Fuzzy_inference()
    #角度と力の大きさを計算
    print('---------Fuzzy----------')
    fx = -req.fz
    fy = -req.fy
    ek = param.old_ek
    print('(fx, fy, ek) = ({}, {}, {})'.format(round(fx,3),round(fy,3),round(ek,3)))
    F = calc.get_force(fx,fy,ek)


    #------------前回の各値を取得----------------
    oldfy = req.old_fy

    with open(param_path) as f:
        reader = csv.reader(f)
   	force = [row for row in reader]
        row = np.shape(force)[0]
    old_x = float(force[row-1][0])
    old_y = float(force[row-1][1])
    old_ek = float(force[row-1][5])

    with open(ForceXY_path) as f:
        reader = csv.reader(f)
   	force = [row for row in reader]
        row = np.shape(force)[0]
    oldFX = float(force[row-1][0])
    oldFY = float(force[row-1][1])
    old_F = [oldFX, oldFY]
    #-------------------------------------------


    
    print('(FX,FY) = ({},{})'.format(round(F[0],3),round(F[1],3)))
    print('(old_FX,old_FY) = ({},{})'.format(round(old_F[0],3),round(old_F[1],3)))


    #Fuzzyの計算
    if  pose_num!=3.0:
        if abs(fy)<3.0:
            x = old_x + fuzzy.fuzzy4position(F[0],F[0]-old_F[0])
            y = old_y + fuzzy.fuzzy4position(F[1],F[1]-old_F[1])
            next_ek = param.old_ek + 0
        if abs(fy)>3.0:
            x = param.old_x + 0
            y = param.old_y + 0
            next_ek = old_ek + fuzzy.fuzzy4angle(fy,fy-oldfy)

        #力の大きさと手渡しパラメータを書き込む
        f= open(param_path, 'a')
        writer = csv.writer(f,lineterminator='\n')
        param_list = [x, y, -0.5, 3.14, -1.57, next_ek]
        writer.writerow(param_list)
        f.close
	f= open(ForceXY_path, 'a')
        writer = csv.writer(f,lineterminator='\n')
        F_list = [F[0],F[1]]
        writer.writerow(F_list)
        f.close


    if  pose_num==3.0:
        f= open(param_path, 'w')
        writer = csv.writer(f,lineterminator='\n')
	x,y,next_ek = -0.5, 0, 0
        param_list = [x, y, -0.5, 3.14, -1.57, next_ek]
        writer.writerow(param_list)
        f.close
    return fuzzyResponse(x,y,next_ek)

if __name__=="__main__":

    ex_result_path = setting.ex_result_path

    rospy.init_node('Fuzzy_server')
    rospy.Service('Fuzzy_Topic', fuzzy, callback)
    param = parameter()
    calc = calculate_force()
    sub_pose = Sub_Pose_estimation()
    while not rospy.is_shutdown():

        rospy.spin()




