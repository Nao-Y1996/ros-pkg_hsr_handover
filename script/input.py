#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from hsr_handing.srv import input4handing
from hsr_handing.msg import grasp_key




if __name__=='__main__':
    rospy.init_node('input')   

    #rospy.wait_for_service('input_topic4grasp')
    #input_client1 = rospy.ServiceProxy('input_topic4grasp', input_str)
    pub = rospy.Publisher('input_topic4grasp', grasp_key, queue_size=10)

    rospy.wait_for_service('input_topic4handing')
    input_client2 = rospy.ServiceProxy('input_topic4handing', input4handing)
    input_key = None
    pub_count = 0
    grasped = False
    while not rospy.is_shutdown():	
       
        try:
            
            
            name = raw_input('Enter the  command --->> ')
               
            if name == 'g' and pub_count==0:
                grasp_key = name
                pub.publish(grasp_key)
                pub_count += 1
                grasped = True
            if name == 'h'and grasped==True:
                
                input_key = name
                response = input_client2(input_key)
                #print(response)
	        grasped = False
                pub_count=0


        except NameError:
            continue 

