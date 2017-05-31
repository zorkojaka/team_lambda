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
import sensor_msgs.msg

from detection_msgs.msg import Detection
from geometry_msgs.msg import Point, Pose, PoseArray, PointStamped, Vector3
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from image_geometry import PinholeCameraModel
from localizer.srv import Localize

import termios

roslib.load_manifest('localizer')


"""
    Tasks of this node:
        -> Face localization in 3d space (with localizer node)
        -> Face location clustering
"""

DIST_THRESH = 0.4
class FaceCluster:
    @staticmethod
    def min_dist_to_cluster(pt, cluster):
        best_dist, curr_dist = 10000.0, 0.0
        for pt2 in cluster:
            curr_dist = math.sqrt(math.pow(pt.x - pt2.x, 2) + math.pow(pt.y - pt2.y, 2))
            best_dist = min(best_dist, curr_dist)
        return best_dist

    def put(self, point):
        best_dist, curr_dist = 10000.0, 0.0
        best_idx = 0

        for i, c in enumerate(self.clusters):
            curr_dist = FaceCluster.min_dist_to_cluster(point, c)
            if curr_dist < best_dist:
                best_dist = curr_dist
                best_idx = i
        
        if best_dist < DIST_THRESH:
            self.clusters[best_idx].append(point)
        else:
            self.clusters.append([])
            self.clusters[-1].append(point)
    
    def get_centers(self):
        cent = []
        for c in self.clusters:
            avg_x, avg_y, avg_z = 0.0, 0.0, 0.0
            for pt in c:
                avg_x += pt.x
                avg_y += pt.y
                avg_z += pt.z
            avg_x /= len(c) * 1.0
            avg_y /= len(c) * 1.0
            avg_z /= len(c) * 1.0
            cent.append(Point(avg_x, avg_y, avg_z))
        return cent

    def __init__(self):
        self.clusters = []

class DetectionMapper():
    def localize_detection(self, detection):
        """
            2d image detection -> 3d point in [map frame]
        """
        u = detection.x + detection.width / 2
        v = detection.y + detection.height / 2

        camera_info = None
        best_time = 100
        for ci in self.camera_infos:
            time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info or best_time > 1:
            return False, None

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(detection.header, point, self.region_scope)

        if not localization:
            return False, None

        point_trans = None

        print "Face localized ", localization.pose.position.x, localization.pose.position.y
        if localization.pose.position.x == 0.0:
            print "Ignoring this one!"
            return False, None
        try:
            #(pos_trans, rot) = self.tf_listener.lookupTransform('/map', detection.header.frame_id, rospy.Time(0))
            point_trans = self.tf_listener.transformPoint('/map', PointStamped(detection.header, localization.pose.position))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

        if point_trans == None: # transformation failed
            return False, None
        
        print "Face detected at %d,%d" % (point_trans.point.x, point_trans.point.y)
        return True, point_trans.point 

    # VISUALIZATION
    def put_marker(self, detection, localization):
        marker = Marker()
        marker.header.stamp = detection.header.stamp
        marker.header.frame_id = detection.header.frame_id
        marker.pose = localization.pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(1)
        marker.id = self.marker_id_counter
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(1, 0, 0, 1)
        self.point_markers.markers.append(marker)
        self.marker_id_counter += 1
  

    # currently unused
    def visualize_centers(self):
        center_markers = MarkerArray()
        centers = self.facecluster.get_centers()
        for i, pnt in enumerate(centers):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = '/map'
            marker.pose.position = pnt
			
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.frame_locked = False
            marker.lifetime = rospy.Duration.from_sec(1)
            
            marker.id = i
            marker.scale = Vector3(0.1, 0.1, 0.1)
            marker.color = ColorRGBA(1, 1, 0, 1)
            
            center_markers.markers.append(marker)

        self.markers_pub.publish(center_markers)

    def publish_centers(self):
        centers = self.facecluster.get_centers()
        center_poses = PoseArray()

        center_poses.header.stamp = rospy.Time.now()
        center_poses.header.frame_id = '/map'
        
        for i, pnt in enumerate(centers):
            pose = Pose()
            pose.position = pnt
            center_poses.poses.append(pose)
        
        self.centers_pub.publish(center_poses)

    def flush(self):
        self.visualize_centers()
        self.publish_centers()
    
    # Callbacks
    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)

    def detections_callback(self, detection):
        sflag, trans_pnt = self.localize_detection(detection)
        if not sflag: # something went wrong
            return
        self.facecluster.put(trans_pnt)
    
    def __init__(self):
        self.region_scope = rospy.get_param('~region', 3)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        
        self.facecluster = FaceCluster()
        rospy.wait_for_service('localizer/localize')

        # Subscribe to topics 
        self.detections_sub = message_filters.Subscriber('detections', Detection)
        self.detections_sub.registerCallback(self.detections_callback)

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.camera_sub = message_filters.Subscriber('camera_info', CameraInfo)
        self.camera_sub.registerCallback(self.camera_callback)
        
        self.localize = rospy.ServiceProxy('localizer/localize', Localize)

        self.tf_listener = tf.TransformListener()

        # Publish on topics
        self.markers_pub = rospy.Publisher('markers', MarkerArray)
        self.centers_pub = rospy.Publisher('centers', PoseArray)

    	self.point_markers = MarkerArray()
        self.marker_id_counter = 0
 
if __name__ == '__main__':

        rospy.init_node('mapper')

        try:
            mapper = DetectionMapper()
            r = rospy.Rate(30)
            while not rospy.is_shutdown():
                mapper.flush()
                r.sleep()
        except rospy.ROSInterruptException: pass
