#!/usr/bin/env python
# Copied from http://www.theconstructsim.com/merge-laser-scans-single-pointcloud/

import rospy
from laser_assembler.srv import AssembleScans2  # Data type for converter service
from sensor_msgs.msg import PointCloud2         # Output Data type

rospy.init_node("laser2pc")
print("Waiting for service assemble_scans2")
rospy.wait_for_service("assemble_scans2")

assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher ("/laser_pointcloud", PointCloud2, queue_size=1)

r = rospy.Rate (1)

while not rospy.is_shutdown():
    try:
        resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
        print "MSG: Got cloud with %u points" % len(resp.cloud.data)
        pub.publish (resp.cloud)

    except rospy.ServiceException, e:
        print "ERR: Service call failed: %s"%e

    r.sleep()
