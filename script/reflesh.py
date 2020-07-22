#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import csv
import os
import time
import glob

file_list = glob.glob('/home/naoyamada/catkin_ws3/src/hsr_handing/script/figure/*.jpeg')
for file in file_list:
    print("remove：{0}".format(file))
    os.remove(file)

path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/force_yz/force_data_stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/force_yz/force_data_sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/force_yz/force_data_cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/force_yz/force_data_unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close


path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/handing_param/handing_param_sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/handing_param/handing_param_stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/handing_param/handing_param_cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/handing_param/handing_param_unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close



path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/Force_XY/sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/Force_XY/stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/Force_XY/cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = '/home/naoyamada/catkin_ws3/src/hsr_handing/script/csv/Force_XY/unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close



