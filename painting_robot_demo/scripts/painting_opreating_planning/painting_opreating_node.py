#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time

import os
class PaintingOpreat():
    def __init__(self):
        pass
    def Init_node(self):
        rospy.init_node("painting_opreator_node")

def main():
    ratet=1
    Aub=PaintingOpreat()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    rospy.set_param('open_control_mobile_platform',1)
    climb_go_up_down_flag=0
    open_stand_bar_flag=0
    w_count=2
    path_num=1
    while not rospy.is_shutdown():
        open_control_mobile_platform=rospy.get_param('open_control_mobile_platform')
        mobile_tracking_stop_flag=rospy.get_param('mobile_tracking_stop_flag')
        climb_distance_tracking_over=rospy.get_param('climb_distance_tracking_over')
        rotation_distance_tracking_over=rospy.get_param('rotation_distance_tracking_over')
        painting_oprea_over=rospy.get_param('painting_oprea_over')
        top_limit_switch_status=rospy.get_param('top_limit_switch_status')
        read_line_encode=rospy.get_param('read_line_encode')
        climb_max_length=rospy.get_param('climb_max_length')
        climb_way_point_length=rospy.get_param('climb_way_point_length')
        # climb_and_stand_bar_rotaion_homing=rospy.get_param('climb_and_stand_bar_rotaion_homing')
        if path_num>0:
            open_stand_bar_flag=0
            if w_count>0:
                
                if mobile_tracking_stop_flag==1:
                    rospy.loginfo("mobile tracking over climb go up----")
                    #enable rotation robot
                    rospy.set_param('enable_control_rotation',1)
                    #enable stand robot
                    rospy.set_param('enable_control_stand_bar',1)
                    #enable climb robot
                    rospy.set_param('enable_climb_control',1)
                    #stand bar
                    if top_limit_switch_status!=1 and open_stand_bar_flag==0:
                        rospy.loginfo("flex pole up-----")
                        rospy.set_param('write_flex_pole_motor_up',1)
                        time.sleep(10)
                        rospy.loginfo("waiting for flex pole go to point")
                        rospy.set_param('distance_control_stand_bar',10)#30cm
                        time.sleep(10)
                        rospy.loginfo("waiting for stand bar go to point")
                        open_stand_bar_flag=1
                    else:
                        rospy.set_param('enable_second_control_stand_bar',1)#reopen stand bar for next point
                    if climb_distance_tracking_over==0:
                        if climb_go_up_down_flag==0:
                            rospy.set_param('distance_climb_control',climb_max_length)#climb 200cm
                            time.sleep(10)
                            rospy.loginfo("waiting for climb go to point")
                            if read_line_encode>=climb_max_length:
                                rospy.set_param('climb_distance_tracking_over',1)
                        else:
                            rospy.set_param('distance_climb_control',-1.0*climb_way_point_length)#climb down 70cm
                            time.sleep(5)
                            rospy.loginfo("waiting for climb go back to start point")
                            if read_line_encode<=(climb_max_length-w_count*climb_way_point_length):
                                rospy.set_param('climb_distance_tracking_over',1)
                                               
                    if climb_distance_tracking_over==1:
                        if rotation_distance_tracking_over==0:
                            rospy.set_param('rad_control_rotation',pi/2)#rotation clockwise pi/2
                            time.sleep(10)
                            rospy.loginfo("waiting for rotation go  to rad point") 
                            rospy.set_param("rotation_distance_tracking_over",1)
                        if rotation_distance_tracking_over==1:
                            rospy.set_param('open_aubo_oprea_flag',1)
                            time.sleep(10)
                            rospy.loginfo("waiting for aubo opreating-----")                             
                            if painting_oprea_over==1:
                                w_count-=1
                                climb_go_up_down_flag=1
                                rospy.set_param('climb_distance_tracking_over',0)
                                rospy.set_param('open_aubo_oprea_flag',0)
                                rospy.set_param('painting_oprea_over',0)
                                if w_count==0:
                                    rospy.set_param('open_control_mobile_platform',1)
                                    rospy.set_param('mobile_tracking_stop_flag',0)
                                    path_num-=1
                                    w_count=2
                                    rospy.set_param('home_climb_flex_bar',1)
                                    time.sleep(15)
                                    rospy.loginfo("waiting for home program over-------")
                                    rospy.set_param('home_climb_flex_bar',0)
               
        else:
            rospy.loginfo("-------all path over-----")
            rospy.set_param('climb_distance_tracking_over',0)
            rospy.set_param('open_aubo_oprea_flag',0)
            rospy.set_param('painting_oprea_over',0)
            rospy.set_param('enable_control_rotation',2)
            rospy.set_param('enable_climb_control',2)
            rospy.set_param('enable_control_stand_bar',2)
            rospy.set_param("home_climb_flex_bar",0)
        rate.sleep()

if __name__ == '__main__':
    main()