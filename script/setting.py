#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import datetime
import rospy
import csv
import time
import glob

# JSTとUTCの差分
DIFF_JST_FROM_UTC = 9#日本時間へ直す
now = datetime.datetime.utcnow() + datetime.timedelta(hours=DIFF_JST_FROM_UTC)
new_dir_name = now.strftime('%Y-%m-%d_%H-%M')


base_path = '/home/naoyamada/catkin_ws3/src/hsr_handing/data'
pos_estimation_path =  base_path + '/ex_result/posture_estimation'
ex_result_path = base_path + '/ex_result/' + new_dir_name


#実験結果保存用のディレクトリ作成
if not os.path.exists(ex_result_path):
    os.mkdir(ex_result_path)
    os.mkdir(ex_result_path+'/figure')
    os.mkdir(ex_result_path+'/force_yz')
    os.mkdir(ex_result_path+'/handing_param')
    os.mkdir(ex_result_path+'/Force_XY')


#センサ座標における力の保存場所
path = ex_result_path + '/force_yz/stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/force_yz/sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path =  ex_result_path + '/force_yz/cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path =  ex_result_path + '/force_yz/unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close

#手渡しパラメータの保存場所（位置や向き）
path =  ex_result_path + '/handing_param/sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path =  ex_result_path + '/handing_param/stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/handing_param/cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/handing_param/unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [-0.5, 0, -0.5, 3.14, -1.57, 0]
writer.writerow(param_list)
f.close

#顔座標における力の保存場所
path = ex_result_path + '/Force_XY/sit.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/Force_XY/stand.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/Force_XY/cross.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
path = ex_result_path + '/Force_XY/unknown.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close



path = ex_result_path + '/dummy.csv'
f= open(path, 'w')
writer = csv.writer(f,lineterminator='\n')
param_list = [0,0]
writer.writerow(param_list)
f.close
