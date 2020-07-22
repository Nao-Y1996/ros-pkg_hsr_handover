#! /usr/bin/env python
# -*- encoding: UTF-8 -*-


import time
#import sys
import csv

PI = 3.1415926535

#読み取りファイルの名前
#fname = "./rt-pose.csv"
fname = '/home/kubotalab/catkin_ws3/src/hsr_handing/script/csv/rt-pose.csv' 
while(1):

	csv_file = open(fname, "r")
	f = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)

	row = f.next()
	print(row[0])
	time.sleep(0.1)

