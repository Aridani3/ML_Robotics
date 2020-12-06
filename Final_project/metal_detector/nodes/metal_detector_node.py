#!/usr/bin/env python
import roslib; roslib.load_manifest('metal_detector')
import rospy
import tf 

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from math import *
import numpy as np

from metal_detector.reader import SensorModel
from metal_detector.particle_filter import ParticleFilter

from visualization_msgs.msg import Marker, MarkerArray

class MetalDetector:
    def __init__(self):
        rospy.init_node('metal_detector')
        self.pose = None
        self.last_pose = Pose2D()
        self.measurement = 0.
        self.filter = None
        self.pf_activated = False
        self.particles = None
        self.bestParticle = None
        self.Treasures = [] # containing [[x, y, uncertainty]...]
        self.world_frame = "/world"
        self.sensor_frame = "/sensor_frame"
        self.Np = 10
        self.min_measurement = .2
        self.min_displacement = .1
        self.markerArray = MarkerArray()
        self.bestParticleMarker = Marker()
        self.min_dis_treasures = 10.0

        self.sub = rospy.Subscriber('~measurements', Float32, self.metalDetector_cb)
        self.listener = tf.TransformListener()
        self.pub_markerArray = rospy.Publisher('~pf_markers', MarkerArray, queue_size=1)
        self.pub_TreasuresArray = rospy.Publisher('~pf_treasures', MarkerArray, queue_size=1)
        self.pub_bp = rospy.Publisher('~pf_bestParticle', Marker, queue_size=1)

        self.world_frame = rospy.get_param("~world_frame", self.world_frame)
        self.sensor_frame = rospy.get_param("~sensor_frame", self.sensor_frame)
        self.Np = rospy.get_param("~Np", self.Np) 
        self.min_measurement = rospy.get_param("~min_measurement", self.min_measurement)
        self.min_displacement = rospy.get_param("~min_displacement", self.min_displacement)
        
        self.Sensor = SensorModel()
        self.Sensor.get_model()
        rospy.loginfo("Sensor model loaded")
        self.sensorModelmean = self.Sensor.sensorModelmean
        self.sensorModelstd = self.Sensor.sensorModelstd

    def metalDetector_cb(self, value):
        self.measurement = value.data
        #rospy.loginfo("Got measurement value of %f", self.measurement)

    def pf_uncertainty(self):
        """
        Uncertainty on the particle filter
        Calculating the sum of weighted distances of particles to best one
        """
        d = self.particles - self.bestParticle.reshape(-1, 1)
        d = np.hypot(d[0], d[1])*self.filter.Weights
        return np.sum(d)

    def updateTreasures(self):
        """
        If the current pf gave a solution better than the existing one
        """
        u = self.pf_uncertainty()
        treasures = np.array(self.Treasures).reshape(len(self.Treasures), 3)
        print("debug: ", treasures[:,:2].T)
        d = treasures[:,:2].T - self.bestParticle.reshape(-1, 1)
        d = np.hypot(d[0], d[1])
        print("distances ", d)
        print("min d: ", np.amin(d))
        if np.amin(d) < self.min_dis_treasures and treasures[np.argmin(d),2] > u:
            treasures[np.argmin(d),:] = list(self.bestParticle) + [u]
            treasures = treasures.reshape(len(self.Treasures), 3)
            self.Treasures = list(treasures)
        if np.amin(d) > self.min_dis_treasures:
            rospy.loginfo("Added new treasure!")
            self.Treasures.append(list(self.bestParticle)+[u])


    def get_MarkerArray(self, poses, size, color):
        markerArray = MarkerArray()
        poses = np.array(poses)
        for i in range(len(poses[0])):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = rospy.get_rostime()
            marker.id = i+1
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = poses[0, i]
            marker.pose.position.y = poses[1, i]
            marker.pose.position.z = 0
            markerArray.markers.append(marker)
        return markerArray

    def get_Marker(self, pose):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.get_rostime()
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0

        return marker

    def Publish(self):
        self.pub_bp.publish(self.bestParticleMarker)
        self.pub_markerArray.publish(self.markerArray)

    def run(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(self.world_frame, self.sensor_frame,
                    rospy.Time(0), rospy.Duration(1.0))
                (pose,rot) = self.listener.lookupTransform(self.world_frame, 
                                self.sensor_frame, rospy.Time(0))
                self.pose = np.array(pose[:2])
                #rospy.loginfo("sensor pose: (%f, %f)", self.pose[0], self.pose[1])
                #rospy.loginfo("particle filter: %s", self.pf_activated)
            except tf.Exception:
                rospy.logerr("TF exception ")


            if self.measurement < self.min_measurement and self.pf_activated:
                self.pf_activated = False
                #rospy.loginfo("Particle filter deactivated")
                
                if self.Treasures == []:
                    self.Treasures.append(list(self.bestParticle)+[self.pf_uncertainty()])
                else:
                    self.updateTreasures()

                self.pub_TreasuresArray.publish(
                    self.get_MarkerArray(np.array(self.Treasures)[:,:2].T, 1, [0, 1, 0]))
                print('Treasures: ', self.Treasures)
                continue

            if not self.pf_activated and self.measurement >= self.min_measurement:
                self.filter = ParticleFilter(self.sensorModelmean, self.sensorModelstd, 
                                            self.pose, self.measurement, Nparticules=self.Np)
                self.particles = list(self.filter.particles)
                self.bestParticle = self.filter.getBestParticle()
                self.markerArray = self.get_MarkerArray(self.particles, 0.05, [1, 0, 0])
                self.bestParticleMarker = self.get_Marker(self.bestParticle)

                self.last_pose.x = self.pose[0]
                self.last_pose.y = self.pose[1]
                self.pf_activated = True
                #rospy.loginfo("Particle filter activated")
                self.Publish()
                continue

            if self.pf_activated and self.measurement >= self.min_measurement:
                dx, dy = self.pose[0]-self.last_pose.x, self.pose[1]-self.last_pose.y
                #rospy.loginfo("dx: %.3f", hypot(dx, dy))
                if hypot(dx, dy) < self.min_displacement:
                    continue

                self.filter.update(self.measurement, self.pose, dx, dy)
                #rospy.loginfo("Particle filter updated")
                self.particles = list(self.filter.particles)
                self.bestParticle = self.filter.getBestParticle()
                self.markerArray = self.get_MarkerArray(self.particles, 0.05, [1, 0, 0])
                self.bestParticleMarker = self.get_Marker(self.bestParticle)
                print("best particle: ", list(self.bestParticle)+[self.pf_uncertainty()])
                self.last_pose.x = self.pose[0]
                self.last_pose.y = self.pose[1]
                self.Publish()


        rate.sleep()

if __name__ == '__main__':
    try:
        metalDetector = MetalDetector()
        metalDetector.run()
        
    except rospy.ROSInterruptException:
        pass
