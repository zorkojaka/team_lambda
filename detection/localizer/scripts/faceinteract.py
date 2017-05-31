#!/usr/bin/env python
import collections
import math
import select
import sys
import tty

import tf

import message_filters
import roslib
import rospy

from geometry_msgs.msg import Point, Pose, PoseArray, PointStamped, Vector3
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from sound_play.libsoundplay import SoundClient

roslib.load_manifest('sound_play')

"""
    Tasks of this node:
        -> Sound interaction if face nearby
"""
class DetectionInteractor():
    
    # Callbacks
    def center_callback(self, center_pose_array):
        if len(center_pose_array.poses) > self.n_detections:
            self.n_detections = len(center_pose_array.poses)
            print "NEW FACE DETECTED!"
            self.soundhandle.say('Hello face!')

    
    def check_position(self):
        try:
			(trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
        
    def check(self):
        self.check_position()

    def __init__(self):
        self.soundhandle = SoundClient()
        rospy.sleep(1)


        # Subscribe to topics 
        self.center_sub = message_filters.Subscriber('centers', PoseArray)
        self.center_sub.registerCallback(self.center_callback)
        self.listener = tf.TransformListener()

        self.n_detections = 0
        
        # Publish on topics
        # self.markers_pub = rospy.Publisher('markers', MarkerArray)
 
if __name__ == '__main__':
        rospy.init_node('faceinteract')

        try:
            interactor = DetectionInteractor()
            
            r = rospy.Rate(30)
            while not rospy.is_shutdown():
                interactor.check()
                r.sleep()
        except rospy.ROSInterruptException: pass
