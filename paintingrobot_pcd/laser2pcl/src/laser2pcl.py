#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('laser_assembler')
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

rospy.init_node("assemble_scans_to_cloud")

rate = rospy.Rate (0.2)

while not rospy.is_shutdown():
    rospy.wait_for_service("assemble_scans2")
    try:
        assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        print "Got cloud with %u points" % len(resp.cloud.data)
        pub = rospy.Publisher ("/assembled_pointcloud", PointCloud2, queue_size=1)
        pub.publish (resp.cloud)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    rate.sleep()
