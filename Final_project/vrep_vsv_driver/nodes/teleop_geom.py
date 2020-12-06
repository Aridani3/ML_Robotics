#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
from math import pi,sqrt
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from vrep_vsv_driver.arm_ik import *

class TeleopIK:
    def __init__(self):
        self.last_joy = -1e10
        self.joy_value = None

        rospy.init_node('vrep_ros_teleop')
        self.sub = rospy.Subscriber('~joy', Joy, self.joy_cb)
        self.twist_sub_h = rospy.Subscriber('~height_controller', Twist, self.h_controller_cb)
        self.twist_sub_l = rospy.Subscriber('~lateral_controller', Twist, self.l_controller_cb)
        self.twist_pub = rospy.Publisher('~twist_command', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher('~position_command', Point, queue_size=1)
        self.tool_pub = rospy.Publisher('~tool_command', Float32, queue_size=1)
        self.axis_arm_x = rospy.get_param("~axis_arm_x",3)
        self.axis_arm_y = rospy.get_param("~axis_arm_y",6)
        self.axis_arm_z = rospy.get_param("~axis_arm_z",4)
        self.arm_step = rospy.get_param("~arm_velocity",0.1)
        self.home_button = rospy.get_param("~home_button",1)
        self.ready_button = rospy.get_param("~ready_button",0)
        self.move_button = rospy.get_param("~move_button",2)
        self.z_control = 0
        self.x_control = 0

        self.arm_distance = None
        self.min_arm_distance = rospy.get_param("~min_arm_distance", 0.8)
        self.max_arm_distance = rospy.get_param("~max_arm_distance", 3.5)
        self.listener = tf.TransformListener()
        self.arm_frame = rospy.get_param("~arm_frame", "/arm_frame")
        self.sensor_frame = rospy.get_param("~sensor_frame", "/sensor_frame")

    def joy_cb(self,value):
        self.last_joy = rospy.rostime.get_time()
        self.joy_value = value

    def h_controller_cb(self, msg):
        self.z_control = msg.linear.z

    def l_controller_cb(self, msg):
        self.x_control = msg.linear.x

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        state = "Ready"
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(self.arm_frame, self.sensor_frame,
                    rospy.Time(0), rospy.Duration(1.0))
                (pose,rot) = self.listener.lookupTransform(self.arm_frame, 
                                self.sensor_frame, rospy.Time(0))
                self.arm_distance = -pose[1]
                rospy.loginfo("distance to arm: %.3f control %.3f", -pose[1], self.x_control)
            except tf.Exception:
                rospy.logerr("TF exception ")

            if (rospy.rostime.get_time() - self.last_joy) < 0.5: 
                if timeout:
                    timeout = False
                    rospy.loginfo("Teleop Geom: Accepting joystick commands")
                if (state == "Homing") or (self.joy_value.buttons[self.home_button]):
                    if state != "Homing":
                        rospy.loginfo("Homing");
                    state = "Homing"
                    self.tool_pub.publish(Float32(0.0))
                    self.pose_pub.publish(Point(0.8,0.0,0.3))
                if (state == "Ready") or (self.joy_value.buttons[self.ready_button]):
                    if state != "Ready":
                        rospy.loginfo("Getting ready");
                    state = "Ready"
                    self.tool_pub.publish(Float32(-pi/2))
                    self.pose_pub.publish(Point(2.0,0.0,-0.5))
                if (state == "Default") or (self.joy_value.buttons[self.move_button]):
                    state = "Default"
                    twist = Twist()
                    if ( self.arm_distance < self.min_arm_distance and self.x_control < 0 ) or ( self.arm_distance > self.max_arm_distance and self.x_control > 0 ) :
                        twist.linear.x = 0.0
                    else: 
                        twist.linear.x = self.x_control
                    rospy.loginfo("lateral control: %f", twist.linear.x)
                    twist.linear.y = self.joy_value.axes[self.axis_arm_y]*self.arm_step
                    twist.linear.z = self.z_control
                    self.tool_pub.publish(Float32(-pi/2))
                    self.twist_pub.publish(twist)
            else:
                if not timeout:
                    timeout = True
                    rospy.loginfo("Teleop Geom: Timeout: ignoring joystick commands")
                    twist = Twist()
                    self.twist_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    try:
        teleop = TeleopIK()
        teleop.run()
        
    except rospy.ROSInterruptException:
        pass
