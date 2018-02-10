#!/usr/bin/env python

import rospy

from tf import TransformListener
from geometry_msgs.msg import PointStamped

class Poses:
    def __init__(self):
        self.tf_listener = TransformListener()

        self.tf_listener.waitForTransform('base_footprint', 'kinect2_depth_optical_frame', rospy.Time(),
                                           rospy.Duration(5.0))

    def transform_point(self, point_stamped, target_frame):
        if point_stamped.header.frame_id == target_frame:
            return point_stamped

        try:
            self.tf_listener.waitForTransform(target_frame, point_stamped.header.frame_id, point_stamped.header.stamp,
                                              rospy.Duration(5.0))

            new_point = self.tf_listener.transformPoint(target_frame, point_stamped)
            return new_point
        except Exception as e:
            print e