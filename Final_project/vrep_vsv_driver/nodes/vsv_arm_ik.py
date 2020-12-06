#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
import copy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist, Vector3, Point
from std_msgs.msg import Float32
from math import pi,sqrt
import tf
from tf.transformations import euler_from_quaternion

from vrep_vsv_driver.arm_ik import *

def sat(val, center, step):
    if val > center + step:
        return center + step
    elif val < center - step:
        return center - step
    else:
        return val

def angle(val, modulo=2*pi):
    return ((val+modulo/2)%modulo)-modulo/2


class VSVArmIK:
    def __init__(self):
        self.ready = False
        self.last_twist = -1e10
        self.twist_value = None
        self.joint_state = None
        self.tool_orientation = -pi/2

        rospy.init_node('vsv_arm_ik')
        self.osub = rospy.Subscriber('~tool_orientation', Float32, self.tool_cb)
        self.tsub = rospy.Subscriber('~twist', Twist, self.twist_cb)
        self.psub = rospy.Subscriber('~position', Point, self.point_cb)
        self.jsub = rospy.Subscriber('~joint_state', JointState, self.joint_cb)
        self.joint = rospy.Publisher('~joint_command', JointState, queue_size=1)
        self.max_velocity = rospy.get_param("~max_velocity",0.1)
        self.name = rospy.get_param("~vsv_name","VSV")
        self.listener = tf.TransformListener()
        rospy.sleep(1.0) # Necessary to let the TF arrive
        while not rospy.is_shutdown() and not self.joint_state:
            rospy.sleep(0.1)
        if rospy.is_shutdown():
            return
        ((a,b,c),_) = self.listener.lookupTransform('/%s/ArmPan'%(self.name),
                '/%s/ArmTilt'%(self.name), rospy.Time(0))
        self.zoffset = c

        u = self.joint_state.position[self.index["ArmExtend"]]
        # Prepare the kinematic solver. The assumption is that
        # extend is at zero
        ((x1,y1,z1),_) = self.listener.lookupTransform('/%s/ArmTilt'%(self.name),
                '/%s/ArmFold'%(self.name), rospy.Time(0))
        print (x1,y1,z1)
        l1 = sqrt(x1*x1+y1*y1+z1*z1)
        ((x2,y2,z2),_) = self.listener.lookupTransform('/%s/ArmFold'%(self.name),
                '/%s/ToolRotate'%(self.name), rospy.Time(0))
        print (x2,y2,z2)
        l2 = sqrt(x2*x2+y2*y2+z2*z2) - u
        rospy.loginfo("Creating kinematic solver: l1=%.3f l2=%.3f"%(l1,l2))
        self.arm_ik = ArmIK(l1,l2)

        self.command = JointState()
        self.command.header = self.joint_state.header
        self.command.name = self.joint_state.name
        self.command.velocity = [0.0] * len(self.joint_state.name)
        self.command.effort = [0.0] * len(self.joint_state.name)
        self.command.position = [x for x in self.joint_state.position]

        ((a,b,c),_) = self.listener.lookupTransform('/%s/ArmTilt'%(self.name),
                '/%s/ToolRotate'%(self.name), rospy.Time(0))
        self.x = -b; self.y = a; self.z = c;
        self.state = "Idle"

    def tool_cb(self,value):
        self.tool_orientation = value.data

    def joint_cb(self,value):
        self.joint_state = value
        self.index = dict(zip(value.name,range(0,len(value.name))))


    def point_cb(self,value):
        if not self.joint_state or not self.ready:
            # Ignoring joystick while we don't have the joint_state
            return
        if self.state != "Position":
            rospy.loginfo("Processing GOTO %.2f %.2f %.2f request",value.x,value.y,value.z)
        value.z -= self.zoffset
        S = self.arm_ik.ik_xyz(value.x,value.y,value.z,0.5)
        if S:
            self.x = value.x; self.y = value.y; self.z = value.z
            (theta0,theta1,theta2,u) = S;
            # print S
            self.state = "Position"
            self.command.position[self.index["ArmPan"]] = theta0
            self.command.position[self.index["ArmTilt"]] = pi/2-theta1
            self.command.position[self.index["ArmFold"]] = -pi/2-theta2
            self.command.position[self.index["ArmExtend"]] = u
        else:
            rospy.loginfo("No Solution")

    def twist_cb(self,value):
        if not self.joint_state or not self.ready:
            # Ignoring joystick while we don't have the joint_state
            return
        if self.state != "Velocity":
            rospy.loginfo("Entering velocity mode")
        self.state = "Velocity"
        self.last_twist = rospy.rostime.get_time()
        self.twist_value = value

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        self.ready = True
        state = "Ready"
        while not rospy.is_shutdown():
            if self.state == "Velocity":
                if (rospy.rostime.get_time() - self.last_twist) < 0.5: 
                    if timeout:
                        timeout = False
                        rospy.loginfo("Accepting twist commands")
                    dt = rate.sleep_dur.to_sec()

                    dx = sat(self.twist_value.linear.x,0,self.max_velocity) * dt
                    dy = sat(self.twist_value.linear.y,0,self.max_velocity) * dt
                    dz = sat(self.twist_value.linear.z,0,self.max_velocity) * dt
                    new_x = self.x + dx
                    new_y = self.y + dy
                    new_z = self.z + dz
                    # rospy.loginfo("Trying to move to %.2f %.2f %.2f from %.2f %.2f %.2f" \
                    #         % (new_x,new_y,new_z, self.x,self.y,self.z))
                    S = self.arm_ik.ik_xyz(new_x,new_y,new_z,0.5)
                    if S:
                        self.x = new_x; self.y = new_y; self.z = new_z
                        (theta0,theta1,theta2,u) = S;
                        # print S
                        self.command.position[self.index["ArmPan"]] = theta0
                        self.command.position[self.index["ArmTilt"]] = pi/2-theta1
                        self.command.position[self.index["ArmFold"]] = -pi/2-theta2
                        self.command.position[self.index["ArmExtend"]] = u
                    else:
                        # rospy.loginfo("No Solution")
                        pass
                else:
                    if not timeout:
                        timeout = True
                        rospy.loginfo("Timeout: ignoring twist commands")
            state = self.joint_state.position[self.index["ToolRotate"]]
            (_,rot) = self.listener.lookupTransform('/%s/Tool'%(self.name),
                    '/%s/ArmPan'%(self.name), rospy.Time(0))
            euler = euler_from_quaternion(rot)
            command = state+angle(euler[0]-self.tool_orientation)
            # print (euler[0], self.tool_orientation, command, state)
            # self.command.position[self.index["ToolRotate"]] += 0.1*error
            self.command.position[self.index["ToolRotate"]] = command
            self.joint.publish(self.command)
            rate.sleep()


if __name__ == '__main__':
    try:
        teleop = VSVArmIK()
        teleop.run()
        
    except rospy.ROSInterruptException:
        pass
