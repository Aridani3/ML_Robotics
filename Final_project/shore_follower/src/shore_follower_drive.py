#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
import numpy as np

class ShoreFollowerDrive:
    def __init__(self):
        self.model_file_ = rospy.get_param("~model_file")
        self.meta_file_ = rospy.get_param("~meta_file")
        self.max_vel_ = rospy.get_param('~max_vel')
        
        self.sess_ = None
        self.load_model()
        
        self.bridge = CvBridge()
        
        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.twist_pub_ = rospy.Publisher("~twist", Twist, queue_size=1)

    def load_model(self):
        # Loads the model
        self.sess_ = tf.Session()
        saver = tf.train.import_meta_graph(self.meta_file_)
        saver.restore(self.sess_, self.model_file_)

    def image_callback(self, data):
        # Image call back and resize to proper shape
        # Regularization is done directly inside the model so we don't have to do it.
        raw = self.bridge.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def sigmoid(self, x, a=15, b=0.8):
        return 1 / (1 + np.exp(-a*(x-b)))

    def image_to_rot(self, img):
        # Reads the image, feed it to the network, get the predictions and act on it.
        out = Twist()
        # Runs the network
        res = self.sess_.run('predictions:0',feed_dict={'is_training:0':False,'drop_prob:0':0.0,'source:0':img})[0]
        # Makes sure that the shape of the network matches the required shape
        assert(res.shape[0] == 2)
        # Using the output of the network the robot is only able to loop around the inner shore anti clock wise (when the robot is already along the shore)
        #print("%5.2f %5.2f" %(res[0],res[1])) 
        # The network never suggest to turn left, so we can't loop clock wise and to loop anti clock wise the robot has to already be along the shore
        """
        if res[0] > 0.5:
            out.linear.x = -0.15
        else:
            out.linear.x = 0.15
        """
        out.linear.x = self.max_vel_* ( self.sigmoid(res[1]) - self.sigmoid(res[0]))
        #print("x", out.linear.x, "y", out.linear.y, "z", out.linear.z, "ax", out.angular.x, "ay", out.angular.y, "az", out.angular.z)
        #TODO: Use the network output so the robot can drive around the lake
        # returns a geometry_msgs.Twist
        return out

if __name__ == '__main__':
        rospy.init_node('shore_follower_drive', anonymous=True)
        fp = ShoreFollowerDrive()
        rospy.spin()

