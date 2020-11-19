#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, Vector3
from math import pi,sqrt
import tf
from tf.transformations import euler_from_quaternion

from vrep_vsv_driver.arm_ik import *

class TeleopIK:
    def __init__(self):
        self.last_joy = -1e10
        self.joy_value = None
        self.joint_state = None

        rospy.init_node('vrep_ros_teleop')
        self.sub = rospy.Subscriber('~joy', Joy, self.joy_cb)
        self.jsub = rospy.Subscriber('~joint_state', JointState, self.joint_cb)
        self.joint = rospy.Publisher('~joint_command', JointState, queue_size=1)
        self.tool = rospy.Publisher('~tool', Vector3, queue_size=1)
        self.name = rospy.get_param("~vsv_name","VSV")
        self.axis_arm_x = rospy.get_param("~axis_arm_x",3)
        self.axis_arm_z = rospy.get_param("~axis_arm_z",4)
        self.axis_arm_y = rospy.get_param("~axis_arm_y",6)
        self.arm_step = rospy.get_param("~arm_step",0.05)
        self.home_button = rospy.get_param("~home_button",1)
        self.ready_button = rospy.get_param("~ready_button",0)
        self.move_button = rospy.get_param("~move_button",2)
        self.listener = tf.TransformListener()
        rospy.sleep(1.0) # Necessary to let the TF arrive
        self.listener.waitForTransform('/%s/ArmTilt'%(self.name),
                '/%s/ArmFold'%(self.name), rospy.Time(0), rospy.Duration(5.0))
        ((x,y,z),_) = self.listener.lookupTransform('/%s/ArmTilt'%(self.name),
                '/%s/ArmFold'%(self.name), rospy.Time(0))
        l1 = sqrt(x*x+y*y+z*z);
        ((x,y,z),_) = self.listener.lookupTransform('/%s/ArmFold'%(self.name),
                '/%s/ArmExtend'%(self.name), rospy.Time(0))
        l2 = sqrt(x*x+y*y+z*z);
        rospy.loginfo("Creating kinematic solver: l1=%.3f l2=%.3f"%(l1,l2))
        self.arm_ik = ArmIK(l1,l2)

    def joint_cb(self,value):
        self.joint_state = value

    def joy_cb(self,value):
        if not self.joint_state:
            # Ignoring joystick while we don't have the joint_state
            return
        self.last_joy = rospy.rostime.get_time()
        self.joy_value = value

    def sat(val, center, step):
        if val > center + step:
            return center + step
        elif val < center - step:
            return center - step
        else:
            return val

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.joint_state:
            rate.sleep()
        if rospy.is_shutdown():
            return
        command = JointState()
        command.header = self.joint_state.header
        command.name = self.joint_state.name
        command.velocity = [0.0] * len(self.joint_state.name)
        command.effort = [0.0] * len(self.joint_state.name)
        command.position = [x for x in self.joint_state.position]
        index = dict(zip(command.name,range(0,len(command.name))))
        ((a,b,c),_) = self.listener.lookupTransform('/%s/ArmPan'%(self.name),
                '/%s/ArmTilt'%(self.name), rospy.Time(0))
        zoffset = c
        ((a,b,c),_) = self.listener.lookupTransform('/%s/ArmTilt'%(self.name),
                '/%s/ToolRotate'%(self.name), rospy.Time(0))
        x = -b; y = a; z = c;
        state = "Ready"
        while not rospy.is_shutdown():
            if (rospy.rostime.get_time() - self.last_joy) < 0.5: 
                if timeout:
                    timeout = False
                    rospy.loginfo("Accepting joystick commands")
                ((a,b,c),_) = self.listener.lookupTransform('/%s/ArmPan'%(self.name),
                        '/%s/ToolRotate'%(self.name), rospy.Time(0))
                c -= zoffset
                self.tool.publish(Vector3(-b,a,c))
                if (state == "Homing") or (self.joy_value.buttons[self.home_button]):
                    if state != "Homing":
                        rospy.loginfo("Homing");
                    state = "Homing"
                    command.position[index["ArmPan"]] = 0.0
                    command.position[index["ArmTilt"]] = 0.0
                    command.position[index["ArmFold"]] = pi/3
                    command.position[index["ArmExtend"]] = 0.0
                    command.position[index["ToolRotate"]] = -pi/3
                if (state == "Ready") or (self.joy_value.buttons[self.ready_button]):
                    if state != "Ready":
                        rospy.loginfo("Getting ready");
                    state = "Ready"
                    command.position[index["ArmPan"]] = 0.0
                    command.position[index["ArmTilt"]] = pi/4
                    command.position[index["ArmFold"]] = 0.0
                    command.position[index["ArmExtend"]] = 0.0
                    command.position[index["ToolRotate"]] = pi/4
                if (state == "Default") or (self.joy_value.buttons[self.move_button]):
                    first = False
                    if state != "Default":
                        first = True
                        x = -b; y = a; z = c;
                        # Prepare the kinematic solver. The assumption is that
                        # extend is at zero
                        ((x1,y1,z1),_) = self.listener.lookupTransform('/%s/ArmTilt'%(self.name),
                                '/%s/ArmFold'%(self.name), rospy.Time(0))
                        print (x1,y1,z1)
                        l1 = hypot(x1,y1)
                        ((x2,y2,z2),_) = self.listener.lookupTransform('/%s/ArmFold'%(self.name),
                                '/%s/ToolRotate'%(self.name), rospy.Time(0))
                        print (x2,y2,z2)
                        l2 = hypot(x2,y2)
                        rospy.loginfo("Creating kinematic solver: l1=%.3f l2=%.3f"%(l1,l2))
                        self.arm_ik = ArmIK(l1,l2)
                        rospy.loginfo("Switching to default behaviour");
                    state = "Default"
                    dx = self.joy_value.axes[self.axis_arm_x]*self.arm_step
                    dy = self.joy_value.axes[self.axis_arm_y]*self.arm_step
                    dz = self.joy_value.axes[self.axis_arm_z]*self.arm_step
                    if first or (abs(dx)>1e-3) or (abs(dy)>1e-3) or (abs(dz)>1e-3):
                        new_x = x + dx
                        new_y = y + dy
                        new_z = z + dz
                        rospy.loginfo("Trying to move to %.2f %.2f %.2f from %.2f %.2f %.2f" \
                                % (new_x,new_y,new_z, x,y,z))
                        S = self.arm_ik.ik_xyz(new_x,new_y,new_z,0.0)
                        if S:
                            x = new_x; y = new_y; z = new_z
                            (theta0,theta1,theta2,u) = S;
                            print S
                            command.position[index["ArmPan"]] = theta0
                            command.position[index["ArmTilt"]] = pi/2-theta1
                            command.position[index["ArmFold"]] = -pi/2-theta2
                            command.position[index["ArmExtend"]] = u
                        else:
                            rospy.loginfo("No Solution")
                    (_,rot) = self.listener.lookupTransform('/%s/Tool'%(self.name),
                            '/%s/ArmPan'%(self.name), rospy.Time(0))
                    euler = euler_from_quaternion(rot)
                    error = (euler[0]+pi/2)
                    # print (euler[0], euler[0]*180./pi, error)
                    command.position[index["ToolRotate"]] += 0.1*error
            else:
                if not timeout:
                    timeout = True
                    rospy.loginfo("Timeout: ignoring joystick commands")
            self.joint.publish(command)
            rate.sleep()


if __name__ == '__main__':
    try:
        teleop = TeleopIK()
        teleop.run()
        
    except rospy.ROSInterruptException:
        pass
