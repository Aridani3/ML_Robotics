#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = -data.axes[0]
    pub.publish(twist)

def teleop_turtle():
    global pub
    pub = rospy.Publisher("/vrep/twistCommand", Twist, queue_size=10)
    rospy.Subscriber("/joy", Joy, callback)

    rospy.init_node('teleop_turtle', anonymous=True)
    rospy.spin(), 
    

if __name__ == '__main__':
    teleop_turtle()


