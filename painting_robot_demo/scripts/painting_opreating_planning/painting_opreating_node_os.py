#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
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
    rospy.set_param('mobile_path_way_planning',1)
    climb_go_up_down_flag=0
    open_stand_bar_flag=0
    w_count=2#rospy.get_param('climb_num_way_point')
    path_num=4#rospy.get_param('path_num_planning')
    # rospy.loginfo("path_num%s",str(path_num))
    climb_minus_count=0
    mobile_path_way_num=1
    close_all_path_flag=0
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
        climb_num_way_point=rospy.get_param('climb_num_way_point')
        path_num_planning=rospy.get_param('path_num_planning')
        rospy.loginfo("path_num_planning%s",str(path_num_planning))
        # climb_and_stand_bar_rotaion_homing=rospy.get_param('climb_and_stand_bar_rotaion_homing')
        if path_num>0:
            rospy.logerr("-------path_num----%d",path_num)
            # os.system('rosparam set /search_port/enable_control_stand_bar 1')
            if w_count>0:
                rospy.logerr("-------w_count----%d",w_count)
                if mobile_tracking_stop_flag==1:
                    # rospy.loginfo("mobile tracking over----")
                    #stand bar
                    if top_limit_switch_status!=1 and open_stand_bar_flag==0:
                        rospy.loginfo("flex pole up-----")
                        # rospy.set_param('write_flex_pole_motor_up',1)
                        os.system('rosparam set /search_port/enable_control_stand_bar 1')
                        time.sleep(0.05)
                        os.system('rosparam set /search_port/enable_control_stand_bar 1')
                        os.system('rosparam set /search_port/enable_control_stand_bar 0')
                        os.system('rosparam set /search_port/write_flex_pole_motor_up 1')
                        time.sleep(0.05)
                        os.system('rosparam set /search_port/write_flex_pole_motor_up 1')
                    
                        time.sleep(10)
                        rospy.loginfo("waiting for flex pole go to point")
                        os.system('rosparam set /search_port/write_flex_pole_motor_up 0')
                        # rospy.set_param('distance_control_stand_bar',10)#30cm
                        os.system('rosparam set /search_port/distance_control_stand_bar 0.15')
                        time.sleep(0.05)
                        os.system('rosparam set /search_port/distance_control_stand_bar 0.15')
                        os.system('rosparam set /search_port/open_hold_flag 1')
                        time.sleep(0.05)
                        os.system('rosparam set /search_port/open_hold_flag 1')
                        
                        time.sleep(20)
                        os.system('rosparam set /search_port/open_hold_flag 0')
                        rospy.loginfo("waiting for stand bar go to point")
                        # initial_line_encode_data=rospy.get_param('read_line_encode')
                        cmd_str='rosparam get /search_port/read_line_encode'
                        (status, output) = commands.getstatusoutput(cmd_str)
                        initial_line_encode_data=float(output)

                        open_stand_bar_flag=1
                        
                        os.system('rosparam set /search_port/enable_control_stand_bar 2')
                        time.sleep(0.05)
                        os.system('rosparam set /search_port/enable_control_stand_bar 2')
                        os.system('rosparam set /search_port/enable_control_stand_bar 0')

                    # else:
                    #     # rospy.set_param('enable_second_control_stand_bar',1)#reopen stand bar for next point
                    #     os.system('rosparam set /search_port/enable_second_control_stand_bar 1')
                    
                    if climb_distance_tracking_over==0:
                        if climb_go_up_down_flag==0:
                            rospy.logerr("climb----to----max point-----")
                            os.system('rosparam set /search_port/enable_climb_control 1')
                            time.sleep(0.1)
                            os.system('rosparam set /search_port/enable_climb_control 1')
                            time.sleep(0.1)
                            os.system('rosparam set /search_port/enable_climb_control 0')
                            # rospy.set_param('enable_climb_control',1)
                            # rospy.set_param('enable_climb_control',0)
                            # rospy.set_param('distance_climb_control',climb_max_length)#climb 200cm
                            tempstr='rosparam set /search_port/distance_climb_control '+str(climb_max_length)
                            rospy.loginfo(tempstr)
                            os.system(tempstr)#('rosparam set /search_port/distance_climb_control 0.1')
                            os.system('rosparam set /search_port/open_climb_flag 1')
                            time.sleep(0.1)
                            os.system('rosparam set /search_port/open_climb_flag 1')
                            
                    
                            time.sleep(10)
                            rospy.loginfo("waiting for climb go to maxlength point")
                            os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                            time.sleep(0.05)
                            os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                            rospy.logerr('climb_max_length+initial_line_encode_data%s',climb_max_length+initial_line_encode_data)
                            if read_line_encode>=(climb_max_length+initial_line_encode_data):
                                # rospy.set_param('climb_distance_tracking_over',1)
                                os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                    
                        if climb_go_up_down_flag==1:
                            os.system('rosparam set /search_port/enable_climb_control 1')
                            time.sleep(0.1)
                            os.system('rosparam set /search_port/enable_climb_control 1')
                            time.sleep(0.1)
                            # rospy.set_param('distance_climb_control',-1.0*climb_way_point_length)#climb down 70cm
                            tempstr='rosparam set /search_port/distance_climb_control '+str(climb_max_length-climb_minus_count*climb_way_point_length)
                            os.system(tempstr)
                            os.system('rosparam set /search_port/open_climb_flag 1')
                            time.sleep(0.05)
                            os.system('rosparam set /search_port/open_climb_flag 1')

                            time.sleep(7)
                            os.system('rosparam set /search_port/open_climb_flag 0')
                            rospy.loginfo("waiting for climb go back to next climb way point")
                            rospy.logerr('climb_max_length+initial_line_encode_data-w_count*climb_way_point_length%s',climb_max_length+initial_line_encode_data-w_count*climb_way_point_length)
                            os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                            time.sleep(0.05)
                            os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                            if read_line_encode<=(climb_max_length+initial_line_encode_data-w_count*climb_way_point_length):
                                # rospy.set_param('climb_distance_tracking_over',1)
                                os.system('rosparam set /search_port/climb_distance_tracking_over 1')
                    
                                               
                    if climb_distance_tracking_over==1:
                        if rotation_distance_tracking_over==0:
                            rospy.loginfo("rotation_distance_tracking-----")
                            os.system('rosparam set /search_port/enable_control_rotation 1')
                            time.sleep(0.05)
                            os.system('rosparam set /search_port/enable_control_rotation 1')
                            os.system('rosparam set /search_port/enable_control_rotation 0')
                            # rospy.set_param('rad_control_rotation',pi/2)#rotation clockwise pi/2
                            os.system('rosparam set /search_port/rad_control_rotation '+str(pi/2))
                            os.system('rosparam set /search_port/open_rotation_flag 1')
                            # os.system('rosparam set /search_port/open_rotation_flag 0')
                            time.sleep(5)
                            rospy.loginfo("waiting for rotation go  to rad point") 
                            # rospy.set_param("rotation_distance_tracking_over",1)
                            os.system('rosparam set /search_port/rotation_distance_tracking_over 1')
                    
                        if rotation_distance_tracking_over==1:
                            # rospy.set_param('open_aubo_oprea_flag',1)
                            if painting_oprea_over==0:
                                
                                if w_count==climb_num_way_point:
                                    rospy.loginfo("aubo go to opreating point-----")
                                    os.system('rosparam set /search_port/aubo_go_back_opreating_point 1')
                                    time.sleep(4)
                                    os.system('rosparam set /search_port/aubo_go_back_opreating_point 0')
                                rospy.loginfo("starting aubo painting opreation-----")
                                os.system('rosparam set /search_port/open_aubo_oprea_flag 1')
                                time.sleep(0.05)
                                os.system('rosparam set /search_port/open_aubo_oprea_flag 1')
                                time.sleep(30)
                                rospy.loginfo("waiting for aubo opreating-----")
                                os.system('rosparam set /search_port/painting_oprea_over 1')
                                os.system('rosparam set /search_port/open_aubo_oprea_flag 0')                            
                            if painting_oprea_over==1:
                                os.system('rosparam set /search_port/painting_oprea_over 0')
                                rospy.loginfo("painting oprea over-----")
                                w_count-=1
                                climb_minus_count+=1
                                climb_go_up_down_flag=1
                                # rospy.set_param('climb_distance_tracking_over',0)
                                os.system('rosparam set /search_port/climb_distance_tracking_over 0')
                    
                                # rospy.set_param('open_aubo_oprea_flag',0)
                                os.system('rosparam set /search_port/open_aubo_oprea_flag 0')
                    
                                # rospy.set_param('painting_oprea_over',0)
                                os.system('rosparam set /search_port/painting_oprea_over 0')
                                time.sleep(0.05)
                                os.system('rosparam set /search_port/painting_oprea_over 0')
                                rospy.loginfo("go to next  opreating task--------")
                                if w_count==0:
                                    # rospy.set_param('open_control_mobile_platform',1)
                                    
                    
                                    # rospy.set_param('mobile_tracking_stop_flag',0)
                                    os.system('rosparam set /search_port/mobile_tracking_stop_flag 0')
                    
                                    path_num-=1
                                    w_count=2
                                    climb_minus_count=0
                                    open_stand_bar_flag=0

                                    # rospy.set_param('home_climb_flex_bar',1)
                                    os.system('rosparam set /search_port/distance_control_stand_bar 0')
                                    os.system('rosparam set /search_port/enable_second_control_stand_bar 1')

                                    os.system('rosparam set /search_port/home_climb_flex_bar 1')
                                    
                                    time.sleep(45)
                                    os.system('rosparam set /search_port/rotation_distance_tracking_over 0')
                                    os.system('rosparam set /search_port/enable_second_control_stand_bar 0')
                                    rospy.loginfo("waiting for home program over-------")
                                    # rospy.set_param('home_climb_flex_bar',0)
                                    os.system('rosparam set /search_port/home_climb_flex_bar 0')
                                    
                                    if mobile_path_way_num>=path_num_planning:
                                        mobile_path_way_num=0
                                    else:
                                        mobile_path_way_num+=1
                                        os.system('rosparam set /search_port/mobile_path_way_planning '+str(mobile_path_way_num))
                                        rospy.logerr("I will go to [ %s ]the mobile_path_way_point-----",str(mobile_path_way_num))
                                    os.system('rosparam set /search_port/open_control_mobile_platform 1')
                                    time.sleep(0.05)
                                    os.system('rosparam set /search_port/open_control_mobile_platform 1')
               
        else:
            rospy.loginfo("-------all path over-----")
            rospy.set_param('climb_distance_tracking_over',0)
            rospy.set_param('open_aubo_oprea_flag',0)
            rospy.set_param('painting_oprea_over',0)
            rospy.set_param('enable_control_rotation',2)
            rospy.set_param('enable_control_rotation',0)
            rospy.set_param('enable_climb_control',2)
            rospy.set_param('enable_climb_control',0)

            rospy.set_param('enable_control_stand_bar',2)
            rospy.set_param('enable_control_stand_bar',0)
            rospy.set_param("home_climb_flex_bar",0)
            if close_all_path_flag==0:
                os.system('rosparam set /search_port/mobile_path_way_planning '+str(0))
                rospy.logerr("I will go to [ %s ]the mobile_path_way_point-----",str(0))
                os.system('rosparam set /search_port/open_control_mobile_platform 0')
                time.sleep(0.05)
                os.system('rosparam set /search_port/open_control_mobile_platform 0')
                close_all_path_flag=1           
        rate.sleep()

if __name__ == '__main__':
    main()