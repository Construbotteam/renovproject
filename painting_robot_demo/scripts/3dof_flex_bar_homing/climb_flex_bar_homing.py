#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time
import os


class FLEX3DOFROBOTHOME():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="climb_flex_bar_home_node"

    hc3dof=FLEX3DOFROBOTHOME(nodename)

    hc3dof.Init_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        home_climb_flex_bar = rospy.get_param("home_climb_flex_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        write_flex_pole_motor_down = rospy.get_param("write_flex_pole_motor_down")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_down'), write_flex_pole_motor_down)
        if home_climb_flex_bar==1:
            os.system('rosparam set /search_port/painting_oprea_over 0')#/search_port/rotation_distance_tracking_over
            os.system('rosparam set /search_port/rotation_distance_tracking_over 0')#/search_port/rotation_distance_tracking_over
            os.system('rosparam set /search_port/climb_distance_tracking_over 0')
            os.system('rosparam set /search_port/open_climb_flag 0')
            rospy.set_param('rad_control_rotation',0)
            os.system('rosparam set /search_port/rad_control_rotation 0')
            os.system('rosparam set /search_port/open_rotation_flag 1')
            time.sleep(8)
            rospy.loginfo("waiting for rotation robot go  to start rad/degree")
            os.system('rosparam set /search_port/distance_climb_control 0')
            rospy.set_param('distance_climb_control',0)
            os.system('rosparam set /search_port/open_climb_flag 1')
            time.sleep(0.05)
            os.system('rosparam set /search_port/open_climb_flag 1')
            time.sleep(6)
            rospy.loginfo("waiting for climb robot go down to start point")
            # rospy.set_param('distance_control_stand_bar',0)
            os.system('rosparam set /search_port/aubo_go_back_initial_point 1')
            time.sleep(4)
            rospy.loginfo("waiting for aubo robot go back to bar point")
            os.system('rosparam set /search_port/aubo_go_back_initial_point 0')
            os.system('rosparam set /search_port/distance_control_stand_bar 0')
            os.system('rosparam set /search_port/enable_control_stand_bar 1')
            time.sleep(0.05)
            os.system('rosparam set /search_port/enable_control_stand_bar 1')
            os.system('rosparam set /search_port/enable_control_stand_bar 0')
            os.system('rosparam set /search_port/distance_control_stand_bar 0')
            time.sleep(0.05)
            os.system('rosparam set /search_port/distance_control_stand_bar 0')
            # rospy.set_param('distance_control_stand_bar',0)
            os.system('rosparam set /search_port/open_hold_flag 1')
            time.sleep(0.05)
            os.system('rosparam set /search_port/open_hold_flag 1')
            time.sleep(15)
            os.system('rosparam set /search_port/enable_control_stand_bar 2')
            os.system('rosparam set /search_port/enable_control_stand_bar 0')
            rospy.loginfo("waiting for stand bar go to start point")
            os.system('rosparam set /search_port/write_flex_pole_motor_down 2')
            time.sleep(0.05)
            os.system('rosparam set /search_port/write_flex_pole_motor_down 2')
            rospy.set_param('write_flex_pole_motor_down',2)
            time.sleep(10)
            rospy.loginfo("waiting for flex bar go down to start point")
            rospy.set_param('write_flex_pole_motor_down',0)

            os.system('rosparam set /search_port/open_climb_flag 0')
            os.system('rosparam set /search_port/write_flex_pole_motor_down 0')

            

            
           
            rospy.set_param('home_climb_flex_bar',0)
            os.system('rosparam set /search_port/home_climb_flex_bar 0')
            os.system('rosparam set /search_port/open_rotation_flag 0')
            os.system('rosparam set /search_port/open_hold_flag 0')
            os.system('rosparam set /search_port/enable_control_rotation 2')
            os.system('rosparam set /search_port/enable_second_control_stand_bar 0')
            os.system('rosparam set /search_port/enable_control_rotation 0')
            os.system('rosparam set /search_port/enable_climb_control 2')
            os.system('rosparam set /search_port/enable_climb_control 0')

            # os.system('rosparam set /search_port/enable_climb_control 2')
            # os.system('rosparam set /search_port/enable_climb_control 0')

            rospy.set_param('mobile_tracking_stop_flag',0)
            os.system('rosparam set /search_port/mobile_tracking_stop_flag 0')
            rospy.loginfo("homing program over--  go to next mobile way point-------")
        rate.sleep()

if __name__=="__main__":
    main()
