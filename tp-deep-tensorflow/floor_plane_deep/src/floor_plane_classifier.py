#!/usr/bin/env python

import rospy

import tensorflow as tf
import numpy as np

from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

class FloorPlaneClassifier:
    def __init__(self):
        self.model_file_ = rospy.get_param("~model_file")
        self.meta_file_ = rospy.get_param("~meta_file")
        self.ts_ = rospy.get_param("~thumb_size")

        self.sess_ = None
        self.load_model()
        
        self.bridge = CvBridge()

        rospy.Subscriber("~image", Image, self.image_callback, queue_size=1)
        self.image_pub_ = rospy.Publisher("~image_label", Image, queue_size=1)

    def load_model(self):
        # Loads the model
        self.sess_ = tf.Session()
        saver = tf.train.import_meta_graph(self.meta_file_)
        saver.restore(self.sess_, self.model_file_)

    def image_callback(self, data):
        # Gets the image, check that it is square and divisible by the thumbnail size
        assert(data.height == data.width)
        assert(data.height%self.ts_ == 0)
        # Convert to np array
        img = self.bridge.imgmsg_to_cv2(data,"bgr8")
        # Reshape the image as a batch of thumbnails (faster processing when using batches)
        batch = np.reshape(np.array(np.split(np.array(np.split(img,self.ts_)),self.ts_)), [-1,self.ts_,self.ts_,3])
        # Calls the network
        checked = self.check_thumb(batch)
        # Transforms the array into low resolution image (on pixel per thumbnail)
        low_res = np.reshape(checked,[data.height/self.ts_,data.width/self.ts_,3])
        # Upsamples the predictions so they have the same size as the input image
        classified = cv2.resize(low_res,(0,0),fx=self.ts_,fy=self.ts_, interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        overlay = cv2.addWeighted(img, 0.5, classified, 0.5, 0)
        # Publish the result
        enc = self.bridge.cv2_to_imgmsg(overlay,"rgb8")
        self.image_pub_.publish(enc)


    def check_thumb(self, batch):
        # Run the network
        res = self.sess_.run('predictions:0',feed_dict={'is_training:0':False,'drop_prob:0':0.0,'source:0':batch})
        # Makes sure that the output has the proper shape
        assert(res[0].shape[0] == 2)
        print(res)
        array = np.zeros((res.shape[0],3))
        amax = np.argmax(res, axis=1)
        for i in range(res.shape[0]):
            array[i,int(amax[i])] = 255.0
        #TODO : Use network output to determine traversability (idealy levaraging vector-wise operation in numpy)
        # Returns a numpy array of dimension [res.shape[0],3]
        return array
           
if __name__ == '__main__':
    rospy.init_node('floor_plane_classifier')
    fp = FloorPlaneClassifier()
    rospy.spin()
