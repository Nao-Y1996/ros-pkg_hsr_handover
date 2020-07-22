#! /usr/bin/env python
# -*- encoding: UTF-8 -*-


import time
import sys
import csv
import rospy
import numpy as np
import os
import math
PI = 3.1415926535

#読み取りファイルの名前
#fname = "./rt-pose.csv"
fname = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/rt-pose.csv' 


dic = { "0x":0, "0y":1, "0reli":2, "1x":3, "1y":4, "1reli":5, 
	"2x":6, "2y":7, "2reli":8, "3x":9, "3y":11, "3reli":11, 
	"4x":12, "4y":13, "4reli":14, "5x":15, "5y":16, "5reli":17, 
	"6x":18, "6y":19, "6reli":20, "7x":21, "7y":22, "7reli":23, 
	"8x":24, "8y":25, "8reli":26, "9x":27, "9y":28, "8reli":29, 
	"10x":30, "10y":31, "10reli":32, "11x":33, "11y":34, "11reli":35, 
	"12x":36, "12y":37, "12reli":38, "13x":39, "13y":40, "13reli":41, 
	"14x":42, "14y":43, "14reli":44, "15x":45, "15y":46, "15reli":47, 
	"16x":48, "16y":49, "16reli":50, "17x":51, "17y":52, "17reli":53, 
	"18x":54, "18y":55, "18reli":56, "19x":57, "19y":58, "19reli":59, 
	"20x":60, "20y":61, "20reli":62, "21x":63, "21y":64, "21reli":65, 
	"22x":66, "22y":67, "22reli":68, "23x":69, "23y":70, "23reli":71, 
	"24x":72, "24y":73, "24reli":74, 
	}

def get_joint(position):
        pos = str(position)     
	x = float(raw[dic[pos+"x"]])
	y = float(raw[dic[pos+"y"]])
	reliability = float(raw[dic[pos+"reli"]])
        return [x,y,reliability]

class parameter():
    def __init__(self):
        self.old_raw = []

if __name__ == '__main__':
	
    param = parameter()

    #時系列データ保存用のファイルを作りなおす
    joint_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/raw-joint.csv'
    f=open(joint_path, 'w')
    f.close
    train_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/nn-train-data.csv'
    f=open(train_path, 'w')
    f.close
    count = 0
    #for i in range(10):
    while(1):
	#MySQLで送られてきたデータで作成されたcsvを読み込む

	with open(fname) as f:
            reader = csv.reader(f)
            for raw in reader:
                data = raw
	'''
	csv_file = open(fname, "r")
	f = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
	names = [row[1] for row in list(reader) if row[1] != ""]
 
	raw_str = f.next()
	csv_file.close
	'''

	time.sleep(0.05)
        raw_str = data
	#数値データに直す
	raw = []
	for i in range(np.shape(raw_str)[0]):
	    raw.append(float(raw_str[i]))

        #新しい数値データが来た時だけ
	if raw!=param.old_raw:
            #時系列データファイルに追記
	    f= open(joint_path, 'a')
	    writer = csv.writer(f,lineterminator='\n')
	    writer.writerow(raw)
	    f.close

            #25*3のデータに整形(25関節, 各関節にx,y,reliability)
	    joint = np.reshape(raw,(25,3))
            
	    #NN用のデータを作る
	    data_list = []
	    #関節間の距離
            def get_len(joint1,joint2):
    	        #if joint1[2]>= 0.3 and joint2[2]>=0.3:
	       	leng = math.sqrt(pow(joint1[0]-joint2[0],2)+pow(joint1[1]-joint2[1],2))
	        #else :
		    #leng = 0
	        return leng
	    #joint1をx軸が通る時のx軸からの角度(0~180°)を返す
            def get_theta(joint1,joint2):
		theta = math.degrees(math.acos((joint2[0]-joint1[0])/get_len(joint1,joint2)))
		return theta
	    #関節のなす角度
            def get_angle(joint1,joint2,joint3):
		angle1 = get_theta(joint1,joint2)
		angle2 = get_theta(joint2,joint3)
                angle = abs(angle1+angle2)
                if angle>180:
                    angle = 360- angle
		return angle
	
            reference = get_len(joint[8],joint[1])
	    if reference != 0:
                data_list.append(get_len(joint[10],joint[1])/reference)
                data_list.append(get_len(joint[13],joint[1])/reference)
                data_list.append(get_len(joint[11],joint[1])/reference)
                data_list.append(get_len(joint[14],joint[1])/reference)
                data_list.append(get_angle(joint[9],joint[10],joint[11]))
                data_list.append(get_angle(joint[12],joint[13],joint[14]))
		data_list.append(get_angle(joint[13],joint[8],joint[1]))
                data_list.append(get_angle(joint[10],joint[8],joint[1]))
                
	    if np.shape(data_list)[0] == 8:
      		#data_listにnanが含まれていないとき書き込む
		if not np.isnan(data_list).any()==True:
		    f= open(train_path, 'a')
		    writer = csv.writer(f,lineterminator='\n')
		    writer.writerow(data_list)
		    f.close
		    #print(data_list)
		    count += 1
		    print(count)
		    if count == 300:
		        break
	    
	param.old_raw = raw









