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

import commands
import re
class LISTPRARMETERS():
    def __init__(self,nodename):
        self.nodename=nodename
        self.param_list=[]
    def Init_node(self):
        rospy.init_node(self.nodename)
    def find_all_param(self,nsname,string):
        self.param_list=re.findall('/'+nsname+'/(.+)\n',string)
    def opreating_cmd(self,ns_name,param_ns):
        cmd_str='rosparam get /'+ns_name+'/'+param_ns
        (status, output) = commands.getstatusoutput(cmd_str)
        rospy.loginfo('/'+ns_name+'/'+param_ns+'--data:-->'+output)
def main():
    nodename="list_parameters_node"

    hc3dof=LISTPRARMETERS(nodename)

    hc3dof.Init_node()
    rate_show_hz=rospy.get_param('list_parameter_hz')
    ns_name=rospy.get_param('ns_name')
    rate = rospy.Rate(rate_show_hz)
    count=0
    while not rospy.is_shutdown():
        (status, output) = commands.getstatusoutput('rosparam list')
        hc3dof.find_all_param(ns_name,output)
        if len(hc3dof.param_list)!=0:
            rospy.logerr("##################loop num:"+str(count)+"###################")
            count+=1
            for i in hc3dof.param_list:
                hc3dof.opreating_cmd(ns_name,i)
        if count>10000:
            count=0
        rate.sleep()

if __name__=="__main__":
    main()
