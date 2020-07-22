#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import csv
import numpy as np
#import rospy
from matplotlib import pyplot as plt
import cv2
import time


def mk_figures():

    csv_read_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/ave_data.csv'
    
        
    with open(csv_read_path) as f:
        reader = csv.reader(f) 
        data = [row for row in reader]
    data.pop(0)#1行目は文字なので削除
    data = [[float(v) for v in row] for row in data]
    print(np.shape(data))
    print(np.shape(data[0]))
    print(np.shape(data[1]))
    print(np.shape(data[2]))
    print(np.shape(data[3]))
    print(np.shape(data[4]))
    print(np.shape(data[5]))
    x = []
    x_ave = []
    y =[]
    y_ave = []
    z = []
    z_ave = []
    
    #x軸の範囲
    x_range = (np.shape(data)[0])
    d_range = x_range-1
    #y軸の値
    for i in range(x_range):
        x.append(data[i][0])
        x_ave.append(data[i][1])
        y.append(data[i][2])
        y_ave.append(data[i][3])
        z.append(data[i][4])
        z_ave.append(data[i][5])

    dx = []        
    for i in range(d_range):
        dx.append(x[i+1]-x[i])

    t_release = np.argmin(dx)
    print(t_release)
    

    #-----------------grasp time の算出-----------------------
    t_grasp_x, t_grasp_y, t_grasp_z = 0,0,0
    #差分から傾き最小点を求める
    count = 0
    for i in range(t_release):
	if (x_ave[t_release-i]-x_ave[t_release-(i+1)])*(x_ave[t_release-(i+1)]-x_ave[t_release-(i+2)])<0 or (x_ave[t_release-i]-x_ave[t_release-(i+1)])*(x_ave[t_release-(i+1)]-x_ave[t_release-(i+2)])==0:
	    t_grasp_x = t_release-(i+1)
            count+=1
	    if count==2:
		break
	else:
	    continue
    count=0
    for i in range(t_release):
	if (y_ave[t_release-i]-y_ave[t_release-(i+1)])*(y_ave[t_release-(i+1)]-y_ave[t_release-(i+2)])<0 or (y_ave[t_release-i]-y_ave[t_release-(i+1)])*(y_ave[t_release-(i+1)]-y_ave[t_release-(i+2)])==0:
	    t_grasp_y = t_release-(i+1)
            count+=1
	    if count==2:
		break
	else:
	    continue
    count=0
    for i in range(t_release):
	if (z_ave[t_release-i]-z_ave[t_release-(i+1)])*(z_ave[t_release-(i+1)]-z_ave[t_release-(i+2)])<0 or (z_ave[t_release-i]-z_ave[t_release-(i+1)])*(z_ave[t_release-(i+1)]-z_ave[t_release-(i+2)])==0:
	    t_grasp_z = t_release-(i+1)
            count+=1
	    if count== 2:
		break
	else:
	    continue
    #t_grasp = int((t_grasp_x + t_grasp_y + t_grasp_z)/3)
    print('{} {} {}'.format(t_grasp_x , t_grasp_y , t_grasp_z))
    #print(t_grasp)
    #---------------------------------------------------------


    
    #x軸の値
    axis_x = np.arange(0,x_range,1)  
    axis_d = np.arange(0,d_range,1)
     

    #=================raw==================
    fig = plt.figure(figsize=(15, 15))
    #センサx方向,グラフの設定
    plt.subplot(3,1,1)
    plt.plot(axis_x, x,color='b')
    plt.plot(axis_x,x_ave,color='r')
    plt.plot(axis_x[t_release], x[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_x], x[t_grasp_x],marker = '.',color = 'g',markersize = 20)
    plt.xlim(0, x_range)
    plt.title('X')
    #センサy方向,グラフの設定
    plt.subplot(3,1,2)
    plt.plot(axis_x, y,color='b')
    plt.plot(axis_x,y_ave,color='r')
    plt.plot(axis_x[t_release], y[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_y], y[t_grasp_y],marker = '.',color = 'g',markersize = 20)
    plt.xlim(0, x_range)
    plt.title('Y')
    #センサz方向,グラフの設定
    plt.subplot(3,1,3)
    plt.plot(axis_x, z,color='b')
    plt.plot(axis_x,z_ave,color='r')
    plt.plot(axis_x[t_release], z[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_z], z[t_grasp_z],marker = '.',color = 'g',markersize = 20)
    plt.xlim(0, x_range)
    plt.title('Z')
    #保存
    figure_name = 'raw.jpeg'
    saving_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/figure/'+ figure_name
    plt.tight_layout()
    title = 'raw data , running average'
    fig.suptitle(title, fontsize=60)
    plt.subplots_adjust(top=0.9)
    plt.savefig(saving_path)
    plt.close()
    #=========================================================
    

    #==================raw_cut==================
    fig = plt.figure(figsize=(10, 15))
     #センサx方向,グラフの設定
    plt.subplot(3,1,1)
    plt.plot(axis_x, x,color='b')
    plt.plot(axis_x,x_ave,color='r')
    plt.plot(axis_x[t_release], x[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_x], x[t_grasp_x],marker = '.',color = 'g',markersize = 20)
    plt.xlim(t_release-200, t_release+200)
    plt.title('X')
    #センサy方向,グラフの設定
    plt.subplot(3,1,2)
    plt.plot(axis_x, y,color='b')
    plt.plot(axis_x,y_ave,color='r')
    plt.plot(axis_x[t_release], y[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_y], y[t_grasp_y],marker = '.',color = 'g',markersize = 20)
    plt.xlim(t_release-200, t_release+200)
    plt.title('Y')
    #センサz方向,グラフの設定
    plt.subplot(3,1,3)
    plt.plot(axis_x, z,color='b')
    plt.plot(axis_x,z_ave,color='r')
    plt.plot(axis_x[t_release], z[t_release],marker = '.',color = 'g',markersize = 20)
    plt.plot(axis_x[t_grasp_z], z[t_grasp_z],marker = '.',color = 'g',markersize = 20)
    plt.xlim(t_release-200, t_release+200)
    plt.title('Z')
    #保存
    figure_name = 'raw_cut.jpeg'
    saving_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/figure/'+ figure_name
    plt.tight_layout()
    title = 'raw data , running average'
    fig.suptitle(title, fontsize=60)
    plt.subplots_adjust(top=0.9)
    plt.savefig(saving_path)
    plt.close()
    #=========================================================

if __name__ == '__main__':
    start = time.time()
    mk_figures()
        
    process_time = time.time() - start
    print('ALL END')
    print('実行時間  :  {}'.format(round(process_time,3)))

