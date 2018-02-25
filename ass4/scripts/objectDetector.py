#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PointStamped


class ObjectDetector:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def find_red_object(self):
        return rospy.wait_for_message(self.topic_name, PointStamped)

