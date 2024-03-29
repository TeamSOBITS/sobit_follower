#! /usr/bin/env python3
#coding:utf-8

import numpy as np
import rospy
# import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector
from multiple_sensor_person_tracking.msg import LegPoseArray

class DrSpaamROS:
    """ROS node to detect pedestrian using DROW3 or DR-SPAAM."""

    def __init__(self):
        self._read_params()
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self._init()

    def _read_params(self):
        """
        @brief      Reads parameters from ROS server.
        """
        self.weight_file = rospy.get_param("~weight_file")
        self.conf_thresh = rospy.get_param("~conf_thresh")
        self.stride = rospy.get_param("~stride")
        self.detector_model = rospy.get_param("~detector_model")
        self.panoramic_scan = rospy.get_param("~panoramic_scan")

    def _init(self):
        """
        @brief      Initialize ROS connection.
        """
        # Publisher
        topic, queue_size, latch = read_publisher_param("detections")
        self._dets_pub = rospy.Publisher(
            topic, LegPoseArray, queue_size=queue_size, latch=latch
        )

        # Subscriber
        topic, queue_size = read_subscriber_param("scan")
        self._scan_sub = rospy.Subscriber(
            topic, LaserScan, self._scan_callback, queue_size=queue_size
        )

    def _scan_callback(self, msg):
        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # t = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        # print("[DrSpaamROS] End-to-end inference time: %f" % (time.time() - t))

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        dets_msg = detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        dets_msg.scan = msg
        # print(dets_msg)
        self._dets_pub.publish(dets_msg)
        # rospy.loginfo("publish LegPoseArray")

def detections_to_pose_array(dets_xy, dets_cls):
    dets_msg = LegPoseArray()
    poses_append = dets_msg.poses.append
    for d_xy, d_cls in zip(dets_xy, dets_cls):
        # Detector uses following frame convention:
        # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
        p = Pose()
        p.position.x = d_xy[0]
        p.position.y = d_xy[1]
        p.position.z = 0.0
        poses_append(p)

    return dets_msg


def read_subscriber_param(name):
    """
    @brief      Convenience function to read subscriber parameter.
    """
    topic = rospy.get_param("~subscriber/" + name + "/topic")
    queue_size = rospy.get_param("~subscriber/" + name + "/queue_size")
    return topic, queue_size


def read_publisher_param(name):
    """
    @brief      Convenience function to read publisher parameter.
    """
    topic = rospy.get_param("~publisher/" + name + "/topic")
    queue_size = rospy.get_param("~publisher/" + name + "/queue_size")
    latch = rospy.get_param("~publisher/" + name + "/latch")
    return topic, queue_size, latch

if __name__ == '__main__':
    rospy.init_node('dr_spaam_ros')
    try:
        DrSpaamROS()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
