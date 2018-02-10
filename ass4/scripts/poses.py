#!/usr/bin/env python

import rospy

from tf import TransformListener

from geometry_msgs.msg import PointStamped, PoseStamped

class Poses:
    def __init__(self):
        self.tf_listener = TransformListener()

        self.tf_listener.waitForTransform('base_footprint', 'kinect2_depth_optical_frame', rospy.Time(),
                                          rospy.Duration(5.0))

        self.tf_listener.waitForTransform('/base_link', '/map', rospy.Time(),
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

    def get_current_pose(self):
        time = self.tf_listener.getLatestCommonTime("/base_link", '/map')
        position, quaternion = self.tf_listener.lookupTransform('/base_link', '/map', time)

        pose = PoseStamped()
        pose.pose.position = position
        pose.pose.orientation = quaternion
        pose.header.frame_id='/map'
        pose.header.stamp = time

        return pose
