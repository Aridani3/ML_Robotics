#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    rospy.loginfo("Message age: %.3f" % (msg.header.stamp - rospy.Time.now()).to_sec())


rospy.init_node('test_clock')
sub = rospy.Subscriber("/vrep/hokuyo",LaserScan,callback,queue_size=1)

rospy.spin()




