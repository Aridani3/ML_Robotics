#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from math import pi

last_joy = -1e10
joy_value = None


def joy_cb(value):
    global joy_value, last_joy
    last_joy = rospy.rostime.get_time()
    joy_value = value


def talker():
    global joy_value
    global last_joy
    rospy.init_node('vsv_teleop_base')
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    pub = rospy.Publisher('~twistCommand', Twist, queue_size=1)
    axis_linear = rospy.get_param("~axis_linear",1)
    axis_angular = rospy.get_param("~axis_angular",0)
    scale_linear = rospy.get_param("~scale_linear",1.)
    scale_angular = rospy.get_param("~scale_angular",1.)
    timeout = True
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist = Twist()
        if (rospy.rostime.get_time() - last_joy) < 0.5: 
            if timeout:
                timeout = False
                rospy.loginfo("Accepting joystick commands")
            twist.linear.x = joy_value.axes[axis_linear] * scale_linear
            twist.angular.z = joy_value.axes[axis_angular] * scale_angular
        else:
            if not timeout:
                timeout = True
                rospy.loginfo("Timeout: ignoring joystick commands")
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
