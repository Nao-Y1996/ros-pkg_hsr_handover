#!/usr/bin/env python
# -*- coding: utf-8 -*-

import mysql.connector
import csv
from time import sleep;


 #csv
#fname = "./rt-pose.csv"
fname ='/home/kubotalab/catkin_ws3/src/hsr_handing/script/csv/rt-pose.csv' 

while(1):
	conn = mysql.connector.connect(host='localhost',
		   user='ext_user',
		   password='mobimobi',
		   database='data_exchange',
		   charset='utf8',
		   buffered=True)
	csv_file = open(fname ,"w")
	writer = csv.writer(csv_file)
	
	#MySQLの操作 execute　メソッドを使用
	cur = conn.cursor()
	cur.execute("select * from data_exchange.pose")

	for row in cur.fetchall():
		writer.writerow(row)
		print(row)

	csv_file.close()

	#接続を閉じる
	cur.close
	conn.close
	
	sleep(1)




