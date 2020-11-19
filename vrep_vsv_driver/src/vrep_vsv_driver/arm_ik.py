#!/usr/bin/env python
import roslib; roslib.load_manifest('vrep_vsv_driver')
import rospy

from math import pi, cos, sin, atan2, acos, asin, sqrt, hypot


class ArmIK:
    def __init__(self, l1, l2):
        self.verbose = False
        self.l1 = l1
        self.l2 = l2
        self.theta0_min = -pi/2
        self.theta0_max = pi/6
        self.theta1_min = 0.
        self.theta1_max = 4*pi/6.
        self.theta2_min = -pi
        self.theta2_max = 0.
        self.u_min = 0.0
        self.u_max = 1.0
        self.u_step = 0.05

    def ik_rzu(self,r, z, u):
        if self.verbose:
            rospy.loginfo("solving IK for %.2f %.2f %.2f" % (r,z,u))
        alpha = atan2(z,r)
        d = hypot(r,z)
        l = self.l2+u
        q = (self.l1*self.l1+d*d-l*l)/(2*self.l1*d);
        if abs(q) > 1:
            if self.verbose:
                rospy.loginfo("invalid acos")
            return None
        beta1 = acos(q);
        theta1 = alpha + beta1;
        if theta1 < self.theta1_min:
            if self.verbose:
                rospy.loginfo("theta1 too small")
            return None
        if theta1 > self.theta1_max:
            if self.verbose:
                rospy.loginfo("theta1 too large")
            return None
        q = (self.l1/l)*sin(beta1);
        if abs(q) > 1:
            if self.verbose:
                rospy.loginfo("invalid asin")
            return None
        beta3 = asin(q);
        beta2 = pi - beta1 - beta3;

        theta2 = -pi + beta2;
        if theta2 < self.theta2_min:
            if self.verbose:
                rospy.loginfo("theta2 too small")
            return None
        if theta2 > self.theta2_max:
            if self.verbose:
                rospy.loginfo("theta2 too large")
            return None
        return (theta1,theta2)

    def ik_rz(self,r,z,alpha=0.5):
        umin = None
        umax = None
        u=self.u_min
        while u <= self.u_max:
            if self.ik_rzu(r,z,u):
                umin = u
                break
            u += self.u_step
        if umin == None:
            if self.verbose:
                rospy.loginfo("No valid U")
            return None
        u = self.u_max
        while u >= self.u_min:
            if self.ik_rzu(r,z,u):
                umax = u
                break
            u -= self.u_step
        u = umin + alpha * (umax-umin)
        (theta1,theta2) = self.ik_rzu(r,z,u)
        return (theta1,theta2,u,umin,umax)

    def ik_xyz(self,x,y,z,alpha=0.5):
        # x towards the grass, y in vehicle forward direction, z up
        if self.verbose:
            rospy.loginfo("Target position %.2f %.2f %.2f"%(x,y,z))
        theta0 = atan2(y,x)
        if theta0 < self.theta0_min:
            if self.verbose:
                rospy.loginfo("Theta0 too small")
            return None
        if theta0 > self.theta0_max:
            if self.verbose:
                rospy.loginfo("Theta0 too large")
            return None
        r = hypot(y,x)
        S = self.ik_rz(r,z, alpha)
        if not S:
            return None
        (theta1,theta2,u,_,_) = S
        return (theta0, theta1, theta2, u)




