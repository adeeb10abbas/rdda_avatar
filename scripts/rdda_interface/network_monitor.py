#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import subprocess

if __name__ == "__main__":
    rospy.init_node("network_monitor")

    rate = rospy.Rate(1)
    host = 'localhost'
    cmd = ['fping', '-c', '3', '-q', host]

    while (not rospy.is_shutdown()):
        resp = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
        rospy.loginfo("Average latency to {0}: ".format(host) + resp.split('/')[-2] + "ms")

    rospy.spin()