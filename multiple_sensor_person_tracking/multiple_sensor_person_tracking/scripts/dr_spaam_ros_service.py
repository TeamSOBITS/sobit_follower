#! /usr/bin/env python3
#coding:utf-8

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector
from multiple_sensor_person_tracking.srv import LegDetection, LegDetectionResponse

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

    def _read_server_param(self, name):
        """
        @brief      Convenience function to read server parameter.
        """
        service = rospy.get_param("~server/" + name + "/service")
        return service

    def _init(self):
        """
        @brief      Initialize ROS connection.
        """
        # server
        service = self._read_server_param("scan")
        self._leg_detect_srv = rospy.Service(service, LegDetection, self._scan_callback)

    def _scan_callback(self, req):
        # todo check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(req.scan.angle_increment * len(req.scan.ranges))
            )
        res = LegDetectionResponse()
        scan = np.array(req.scan.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # t = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        # print("[DrSpaamROS] End-to-end inference time: %f" % (t - time.time()))

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        res.leg_pose_array = self._detections_to_pose_array(dets_xy, dets_cls)
        res.leg_pose_array.header = req.scan.header

        return res

    def _detections_to_pose_array(self, dets_xy, dets_cls):
        pose_array = PoseArray()
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            # Detector uses following frame convention:
            # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
            p = Pose()
            p.position.x = d_xy[0]
            p.position.y = d_xy[1]
            p.position.z = 0.0
            pose_array.poses.append(p)

        return pose_array

if __name__ == '__main__':
    rospy.init_node('dr_spaam_ros_service')
    try:
        DrSpaamROS()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()