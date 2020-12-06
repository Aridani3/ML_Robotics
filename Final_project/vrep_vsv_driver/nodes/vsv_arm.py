#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist,PoseStamped, Quaternion
from math import atan2, hypot, pi, cos, sin
import tf
import message_filters

from vrep_vsv_driver.vsv_kinematics import *


class VSVDriver:
    def __init__(self,name):
        self.name = name
        rospy.init_node('vsv_arm')
        self.name = rospy.get_param("~vsv_name",self.name)
        rospy.loginfo("Starting vsv arm for '%s'" % self.name)
        self.last_cmd = rospy.Time.now()
        self.joint_pub=rospy.Publisher("/%s/aggregated/state"%self.name,JointState, queue_size=1)
        self.command_sub=rospy.Subscriber("/%s/aggregated/command"%self.name,JointState,self.sync_joint_cmd)
        self.joint_sub={}
        self.command_pub={}
        self.ready = False
        self.connected = False
        self.joint_names = [ "ArmPan",  "ArmTilt",  "ArmFold",  "ArmExtend",  "ToolRotate"]
        self.min_value = {"ArmPan":-pi/2, "ArmTilt":-pi/6, "ArmFold":-pi/2,
                "ArmExtend":0.0, "ToolRotate":-pi/2}
        self.max_value = {"ArmPan":pi/2, "ArmTilt":pi/2, "ArmFold":pi/2,
                "ArmExtend":1.0, "ToolRotate":pi/2}

        # print "Initialising wheel data structure"
        for k in self.joint_names:
            self.command_pub[k] = rospy.Publisher("/%s/%s/command" % (self.name,k), Float64, queue_size=1)
            self.joint_sub[k] = message_filters.Subscriber("/%s/%s/state" % (self.name,k), JointState)
            # print "Initialised wheel " + k
        self.ts = message_filters.TimeSynchronizer(self.joint_sub.values(), 10)
        self.ts.registerCallback(self.sync_joint_cb)


    def sync_joint_cb(self,*args):
        self.connected = True
        js = JointState()
        js.header = args[0].header
        for j in args:
            js.name+=j.name
            js.position+=j.position
            js.velocity+=j.velocity
            js.effort+=j.effort
        self.joint_pub.publish(js)

    def sat(self,name,value):
        return max(self.min_value[name],min(self.max_value[name],value))

    def sync_joint_cmd(self,js):
        for (name,value) in zip(js.name,js.position):
            self.command_pub[name].publish(self.sat(name,value))


    def run(self):
        timeout = True
        rate = rospy.Rate(2)
        rospy.loginfo("VSV Arm: We're ready")
        rospy.spin()

if __name__ == '__main__':
    try:
        rd = VSVDriver("VSV") 
        rd.run()
    except rospy.ROSInterruptException:
        pass
