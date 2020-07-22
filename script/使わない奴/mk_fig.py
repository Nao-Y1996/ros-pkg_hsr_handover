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
    
    
    W_x = []
    W_x_ave = []
    W_y =[]
    W_y_ave = []
    W_z = []
    W_z_ave = []
    
    #x軸の範囲
    x_range = (np.shape(data)[0])

    #y軸の値
    for i in range(x_range):
        W_x.append(data[i][0])
        W_x_ave.append(data[i][1])
        W_y.append(data[i][2])
        W_y_ave.append(data[i][3])
        W_z.append(data[i][4])
        W_z_ave.append(data[i][5])

    #x軸の値
    x = np.arange(0,x_range,1)  
    

    #==================3軸まとめたグラフの作成======================
    fig = plt.figure(figsize=(15, 15))
    #センサx方向,グラフの設定
    plt.subplot(3,1,1)
    plt.plot(x, W_x,color='b')
    plt.plot(x,W_x_ave,color='r')
    plt.xlim(0, x_range)
    plt.title('X')
    #センサy方向,グラフの設定
    plt.subplot(3,1,2)
    plt.plot(x, W_y,color='b')
    plt.plot(x,W_y_ave,color='r')
    plt.xlim(0, x_range)
    plt.title('Y')
    #センサz方向,グラフの設定
    plt.subplot(3,1,3)
    plt.plot(x, W_z,color='b')
    plt.plot(x,W_z_ave,color='r')
    plt.xlim(0, x_range)
    plt.title('Z')
    #保存
    figure_name = 'raw_ave.jpeg'
    saving_path = 'figure/'+ figure_name
    plt.tight_layout()
    title = 'raw data , running average'
    fig.suptitle(title, fontsize=60)
    plt.subplots_adjust(top=0.9)
    plt.savefig(saving_path)
    plt.close()
    #=========================================================





    #==================差分を3軸まとめたグラフの作成======================
    fig = plt.figure(figsize=(15, 15))
    #センサx方向,グラフの設定
    plt.subplot(3,1,1)
    plt.plot(x, (np.array(W_x) - np.array(W_x_ave)),color='b')
    #plt.plot(x,W_x_ave,color='r')
    plt.xlim(x_range-1500, x_range-500)
    plt.title('X')
    #センサy方向,グラフの設定
    plt.subplot(3,1,2)
    plt.plot(x, (np.array(W_y) - np.array(W_y_ave)) ,color='b')
    #plt.plot(x,W_y_ave,color='r')
    plt.xlim(x_range-1500, x_range-500)
    plt.title('Y')
    #センサz方向,グラフの設定
    plt.subplot(3,1,3)
    plt.plot(x, (np.array(W_z) - np.array(W_z_ave)) ,color='b')
    #plt.plot(x,W_z_ave,color='r')
    plt.xlim(x_range-1500, x_range-500)
    plt.title('Z')
    #保存
    figure_name = 'difference.jpeg'
    saving_path = 'figure/'+figure_name
    plt.tight_layout()
    title = 'raw data - running average'
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

