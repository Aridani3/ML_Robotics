#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from math import pi

last_joy = -1e10
joy_value = None
joint_state = None

def joint_cb(value):
    global joint_state
    joint_state = value

def joy_cb(value):
    global joy_value, last_joy, joint_state
    if not joint_state:
        # Ignoring joystick while we don't have the joint_state
        return
    last_joy = rospy.rostime.get_time()
    joy_value = value

def sat(val, center, step):
    if val > center + step:
        return center + step
    elif val < center - step:
        return center - step
    else:
        return val

def talker():
    global joy_value
    global last_joy
    global joint_state
    rospy.init_node('vsv_arm_teleop_raw')
    sub = rospy.Subscriber('~joy', Joy, joy_cb)
    jsub = rospy.Subscriber('~joint_state', JointState, joint_cb)
    joint = rospy.Publisher('~joint_command', JointState, queue_size=1)
    axis={}
    axis["ArmPan"] = [int(s) for s in str(rospy.get_param("~axis_pan","6")).split(",")]
    axis["ArmTilt"] = [int(s) for s in str(rospy.get_param("~axis_tilt","7")).split(",")]
    axis["ArmFold"] = [int(s) for s in str(rospy.get_param("~axis_fold","4,5")).split(",")]
    axis["ArmExtend"] = [int(s) for s in str(rospy.get_param("~axis_extend","0,3")).split(",")]
    axis["ToolRotate"] = [int(s) for s in str(rospy.get_param("~axis_rotate","2,1")).split(",")]
    step_value = {"ArmPan":1*pi/180., "ArmTilt":5*pi/180.,
            "ArmFold":2*pi/180., "ArmExtend":0.05, "ToolRotate":5*pi/180.}
    timeout = True
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not joint_state:
        rate.sleep()
    if rospy.is_shutdown():
        return
    command = JointState()
    command.header = joint_state.header
    command.name = joint_state.name
    command.velocity = [0.0] * len(joint_state.name)
    command.effort = [0.0] * len(joint_state.name)
    command.position = [x for x in joint_state.position]
    index = dict(zip(command.name,range(0,len(command.name))))
    while not rospy.is_shutdown():
        if (rospy.rostime.get_time() - last_joy) < 0.5: 
            if timeout:
                timeout = False
                rospy.loginfo("Accepting joystick commands")
            for k in axis.keys():
                if len(axis[k])==1:
                    # Axis
                    command.position[index[k]] = sat(command.position[index[k]]+joy_value.axes[axis[k][0]]*step_value[k],
                            joint_state.position[index[k]],step_value[k])
                else:
                    # Buttons
                    if joy_value.buttons[axis[k][0]]:
                        command.position[index[k]] = sat(command.position[index[k]]-step_value[k],
                            joint_state.position[index[k]],step_value[k])
                    elif joy_value.buttons[axis[k][1]]:
                        command.position[index[k]] = sat(command.position[index[k]]+step_value[k],
                            joint_state.position[index[k]],step_value[k])

            # if twist.linear.x < 0:
            #     twist.angular.z = - twist.angular.z
        else:
            if not timeout:
                timeout = True
                rospy.loginfo("Timeout: ignoring joystick commands")
        joint.publish(command)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
