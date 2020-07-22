#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation

import setting
import math
import os
import sys
import csv
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import WrenchStamped
import rospy
from sensor_msgs.msg import JointState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal
)
from tmc_manipulation_msgs.srv import (
    SafeJointChange,
    SafeJointChangeRequest
)

import matplotlib 
matplotlib.use('Agg') 
from matplotlib import pyplot as plt
import RunAve_fil
#import fuzzy
from std_msgs.msg import Bool
from hsr_handing.msg import grasp_key
from hsr_handing.srv import fuzzy
from hsr_handing.msg import pose_info
from hsrb_interface import geometry
from hsrb_interface import Robot
robot = Robot()
omni_base = robot.try_get('omni_base')
whole_body = robot.try_get('whole_body')
gripper = robot.get('gripper')

_CONNECTION_TIMEOUT = 10.0
# Move timeout[s]
_MOVE_TIMEOUT=60.0
# Grasp force[N]
_GRASP_FORCE=0.2
# TF name of the bottle
_BOTTLE_TF='ar_marker/150'
# TF name of the gripper
_HAND_TF='hand_palm_link'

# Posture that 0.04[m] front and rotate -1.57 around z-axis of the bottle maker
bottle_to_hand = geometry.pose(z=-0.04, ek=-1.57)
# Posture to move the hand 0.02[m] up
hand_up = geometry.pose(x=0.02)
# Posture to move the hand 0.5[m] back
hand_back = geometry.pose(z=-0.5)



# Calcurate running_average
_AVERAGE_RANGE = 200
_x_elements=[0]*_AVERAGE_RANGE
_y_elements=[0]*_AVERAGE_RANGE
_z_elements=[0]*_AVERAGE_RANGE
def running_average(elements,row_deta):

    for i in range(_AVERAGE_RANGE-1):
        elements[i] = elements[i+1]

    elements[_AVERAGE_RANGE-1] = row_deta

    return round(sum(elements)/_AVERAGE_RANGE,3)

class ForceSensorCapture(object):
    def __init__(self):
        self._force_data_x = 0.0
        self._force_data_y = 0.0
        self._force_data_z = 0.0
        #ft_sensor_topic = '/hsrb/wrist_wrench/raw'
        ft_sensor_topic = '/hsrb/wrist_wrench/compensated'
        self._wrist_wrench_sub = rospy.Subscriber(ft_sensor_topic, WrenchStamped, self.__ft_sensor_cb)
        try:
            rospy.wait_for_message(ft_sensor_topic, WrenchStamped,timeout=20)
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
    #CallBack関数
    def __ft_sensor_cb(self, data):
        self._force_data_x = data.wrench.force.x
        self._force_data_y = data.wrench.force.y
        self._force_data_z = data.wrench.force.z

    def get_current_force(self):
        return [self._force_data_x, self._force_data_y, self._force_data_z]



class JointController(object):
    #"""Control arm and gripper"""
    def __init__(self):
        joint_control_service = '/safe_pose_changer/change_joint'
        grasp_action = '/hsrb/gripper_controller/grasp'
        self._joint_control_client = rospy.ServiceProxy(joint_control_service, SafeJointChange)
        self._gripper_control_client = actionlib.SimpleActionClient(grasp_action, GripperApplyEffortAction)
        # Wait for connection
        try:
            self._joint_control_client.wait_for_service(timeout=_CONNECTION_TIMEOUT)
            if not self._gripper_control_client.wait_for_server(rospy.Duration(_CONNECTION_TIMEOUT)):
                raise Exception(grasp_action + ' does not exist')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)
    def move_to_joint_positions(self, goal_joint_states):
        """Joint position control"""
        try:
            req = SafeJointChangeRequest(goal_joint_states)
            res = self._joint_control_client(req)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return False
        return res.success
    def grasp(self, effort):
        """Gripper torque control"""
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        # Send message to the action server
        if (self._gripper_control_client.send_goal_and_wait(goal) ==GoalStatus.SUCCEEDED):
            return True
        else:
            return False
    
class parameter():
    def __init__(self):       
        
	self.handing_count = 0
        self.old_fy = 0
        self.old_fz = 0

class Sub_Pose_estimation():
    def __init__(self):
        rospy.Subscriber("pose_topic", pose_info, self.callback)
	self.pose = 3
    def callback(self, data):
        self.pose = data.pose_identification_number
'''
def Fuzzy_Client():

    rospy.wait_for_service('Fuzzzy_Topic')
    service = rospy.ServiceProxy('Fuzzy_Topic', fuzzy)

    num = 1
    response = service(num)

    print('x={},y={},ek={}'.format(response.x,response.y,response.ek))
'''
  

def gripper_operating(data):

    loop = 0
    loop2 = 0
    grasp = False
    Released = False
    Got_all_data = False
    Calculated = False
    saved=False
    
    key = data.grasp_key 
    #count_param = count_parameter()
    if key == 'g':
        grasp = True
	param.handing_count +=1
	print('\n===================')
	print(str(param.handing_count)+'回目')
	print('===================')
        raw = []
        ave = []

        x = []
        x_ave = []
        y =[]
        y_ave = []
        z = []
        z_ave = []

	sensor_path = ex_result_path + '/sensor_data'+str(param.handing_count)+'.csv'
        f= open(sensor_path, 'w')
        f.close
	
    while Got_all_data == False and grasp == True:
        loop += 1  



        pre_force_list = force_sensor_capture.get_current_force()
        x_run_ave = running_average(_x_elements,pre_force_list[0])
        #y_run_ave = running_average(_y_elements,pre_force_list[1])
        #z_run_ave = running_average(_z_elements,pre_force_list[2])
        #data_list = [pre_force_list[0],x_run_ave,pre_force_list[1],y_run_ave,pre_force_list[2],z_run_ave]
        data_list = [pre_force_list[0],pre_force_list[1],pre_force_list[2]]
        rospy.sleep(0.002)
        #save data
        f= open(sensor_path, 'a')
        writer = csv.writer(f,lineterminator='\n')
        writer.writerow(data_list)
        f.close

        #grasp
        if (loop == 500):
            # 
	    '''
            omni_base.go_abs(0.0, 0.0, -1.5, 300.0)
            whole_body.move_to_neutral()
            # Look at the hand after the transition
            whole_body.looking_hand_constraint = False
            # Move the hand to front of the bottle
            whole_body.move_end_effector_pose(bottle_to_hand, _BOTTLE_TF)
            # Specify the force to grasp
            gripper.apply_force(_GRASP_FORCE)
            # Move the hand up on end effector coordinate
            whole_body.move_end_effector_pose(hand_up, _HAND_TF)
            # Move the hand back on end effector coordinate
            whole_body.move_end_effector_pose(hand_back, _HAND_TF)
            # Transit to initial posture
            omni_base.go_abs(0.0, 0.0, 0.0, 300.0)
	    #whole_body.move_to_neutral()
	    whole_body.move_to_joint_positions({'head_tilt_joint': 0.2})
	    joint_controller.grasp(-0.05)
	    '''
            
	    gripper.apply_force(_GRASP_FORCE)


           #--------------Handing-----------------#

	    
        #release
        #if ((x_run_ave*0.7>pre_force_list[0])and(loop>500)and(Released==False)and(sub_flag.flag==True)):
	if ((x_run_ave*0.4>pre_force_list[0])and(loop>500)and(Released==False)):
            joint_controller.grasp(0.0)
            joint_controller.move_to_joint_positions(release)
	    joint_controller.move_to_joint_positions(initial_position)
	    
            Released = True

        if Released == True:
            loop2 += 1    
            #データ書き込み終了
        if loop2 > 1000:
            Got_all_data = True     
	    omni_base.go_abs(0.0, 0.0, 0.0, 300.0)#初期位置に戻る
	    whole_body.move_to_neutral()
            break


        #移動平均の計算と書き込み
    loop = 0

    while Got_all_data == True and Calculated == False:   
	loop += 1    
	#param.handing_count +=1       
        with open(sensor_path) as f:
            reader = csv.reader(f)
   	    raw = [row for row in reader]
	#rawデータの取得
        raw = [[float(v) for v in row] for row in raw]
        raw_data_length = np.shape(raw)[0]
	#aveデータの取得
        ave = RunAve_fil.running_average_filter(7,raw,raw_data_length) 
        for i in range(4): 		
            ave = RunAve_fil.running_average_filter(7,ave, raw_data_length)

        #x軸の範囲
        x_range = raw_data_length
        #print('data range = {}'.format(x_range))
        d_range = x_range-1
        #y軸の値
        for i in range(x_range):
            x.append(raw[i][0])
            x_ave.append(ave[i][0])
            y.append(raw[i][1])
            y_ave.append(ave[i][1])
            z.append(raw[i][2])
            z_ave.append(ave[i][2])

	#--------ralease timeの算出------------
        dx = []        
        for i in range(d_range):
            dx.append(x[i+1]-x[i])
        t_release = np.argmin(dx)
        #print('Tr = {}'.format(t_release))
        #--------------------------------------

        #-----------------grasp time の算出-----------------------
        t_grasp_x, t_grasp_y, t_grasp_z = 0,0,0
        #差分から傾き最小点を求める
        count = 0
        for i in range(t_release):
            if (x_ave[t_release-30-i]-x_ave[t_release-30-(i+1)])*(x_ave[t_release-30-(i+1)]-x_ave[t_release-30-(i+2)])<=0 :
                t_grasp_x = t_release-(i+1)
                count+=1
                if count==1:
                    break
            else:
                continue
        count=0
        for i in range(t_release):
            if (y_ave[t_release-30-i]-y_ave[t_release-30-(i+1)])*(y_ave[t_release-30-(i+1)]-y_ave[t_release-30-(i+2)])<=0 :
                t_grasp_y = t_release-(i+1)
                count+=1
                if count==1:
                    break
            else:
                continue
        count=0
        for i in range(t_release):
            if (z_ave[t_release-30-i]-z_ave[t_release-30-(i+1)])*(z_ave[t_release-30-(i+1)]-z_ave[t_release-30-(i+2)])<=0 :
                t_grasp_z = t_release-(i+1)
                count+=1
                if count== 1:
                    break
            else:
                continue
        #print('(Tg_y,Tg_z) = {} {}'.format(t_grasp_y , t_grasp_z))
        #---------------------------------------------------------


        fy = y[t_release] - y[t_grasp_y]
	fz = z[t_release] - z[t_grasp_z]
        print('   ')
	#print('---------Fuzzy----------')
        #print('(fy, fz) = {}, {}'.format(round(fy,3),round(fz,3)))

        pose_num = sub_pose.pose
        if pose_num == 0.0:
            force_path = ex_result_path+'/force_yz/sit.csv'
        if pose_num == 1.0:
            force_path = ex_result_path+'/force_yz/stand.csv'
        if pose_num == 2.0:
            force_path = ex_result_path+'/force_yz/cross.csv'
        if pose_num == 3.0:
            force_path = ex_result_path+'/force_yz/unknown.csv'
        

	Calculated = True
	#前回のfを取得
        with open(force_path) as f:
            reader = csv.reader(f)
   	    force = [row for row in reader]
        row = np.shape(force)[0]
        old_fy = float(force[row-1][0])#最後の行を読む
        old_fz = float(force[row-1][1])

        #----------fuzzyを計算する---------------
        response = service(fy,fz,old_fy,old_fz)
	#---------------------------------------

	#新たに今回のfを書き込む
	f= open(force_path, 'a')
        writer = csv.writer(f,lineterminator='\n')
        force_data_list = [fy,fz]
        writer.writerow(force_data_list)
        f.close
        print('next--> x,y,ek={},{},{}'.format(round(response.x,3),round(response.y,3),round(math.degrees(response.ek),3)))
  
#------------------------------------------------------------------------------#
	#param.old_fy = fy 
	#param.old_fz = fz 

        
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
        #plt.plot(axis_x[t_grasp_x], x[t_grasp_x],marker = '.',color = 'g',markersize = 20)
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
        figure_name = 'raw'+str(param.handing_count)+'.jpeg'
        saving_path = ex_result_path+'/figure/'+ figure_name
        plt.tight_layout()
        title = 'wrist sensor data'
        fig.suptitle(title, fontsize=50)
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
        #plt.plot(axis_x[t_grasp_x], x[t_grasp_x],marker = '.',color = 'g',markersize = 20)
        plt.xlim(t_release-300, t_release+100)
        plt.title('X')
        #センサy方向,グラフの設定
        plt.subplot(3,1,2)
        plt.plot(axis_x, y,color='b')
        plt.plot(axis_x,y_ave,color='r')
        plt.plot(axis_x[t_release], y[t_release],marker = '.',color = 'g',markersize = 20)
        plt.plot(axis_x[t_grasp_y], y[t_grasp_y],marker = '.',color = 'g',markersize = 20)
        plt.xlim(t_release-300, t_release+100)
        plt.title('Y')
        #センサz方向,グラフの設定
        plt.subplot(3,1,3)
        plt.plot(axis_x, z,color='b')
        plt.plot(axis_x,z_ave,color='r')
        plt.plot(axis_x[t_release], z[t_release],marker = '.',color = 'g',markersize = 20)
        plt.plot(axis_x[t_grasp_z], z[t_grasp_z],marker = '.',color = 'g',markersize = 20)
        plt.xlim(t_release-300, t_release+100)
        plt.title('Z')
        #保存
        figure_name = 'raw_cut'+str(param.handing_count)+'.jpeg'
        saving_path = ex_result_path+'/figure/'+ figure_name
        plt.tight_layout()
        title = 'wrist sensor data'
        fig.suptitle(title, fontsize=30)
        plt.subplots_adjust(top=0.9)
        plt.savefig(saving_path)
        plt.close()
        #=========================================================

        del data_list[:]
        del raw[:]
        del ave[:]

        del x[:]
        del x_ave[:]
        del y[:]
        del y_ave[:]
        del z[:]
        del z_ave[:]


        if loop==1:
            
            saved=True
            grasp = False
	    #print('maked figure')
            #print('saved all data')
            rospy.sleep(1)
            break

        
        
if __name__ == '__main__':
    try: 

        #rospy.init_node('gripper_node')
        #spin_rate = rospy.Rate(1)
        # 各クラスのインスタンス化
        force_sensor_capture = ForceSensorCapture()
        joint_controller = JointController()
        param = parameter()
	sub_pose = Sub_Pose_estimation()
        #sub_flag = Sub_handing_end_flag()
        initial_position = JointState()
        initial_position.name.extend(['arm_lift_joint', 'arm_flex_joint',
                                  'arm_roll_joint', 'wrist_flex_joint',
                                  'wrist_roll_joint', 'hand_motor_joint'])
        initial_position.position.extend([0.0, 0.0, 0.0, -1.57,
                                      0.0, 1.2])
   
        release = JointState()
        release.name.extend(['hand_motor_joint'])
        release.position.extend([1.1])

	ex_result_path = setting.ex_result_path

        #rospy.wait_for_service('Fuzzzy_Topic')
        service = rospy.ServiceProxy('Fuzzy_Topic', fuzzy)
	

        while not rospy.is_shutdown():
	    rospy.Subscriber("input_topic4grasp", grasp_key, gripper_operating)
            rospy.spin()


    except rospy.ROSException as wait_for_msg_exception:
        rospy.logerr(wait_for_msg_exception)



