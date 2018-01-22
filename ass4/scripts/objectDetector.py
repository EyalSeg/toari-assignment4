#!/usr/bin/env python

import rospy

from promise import Promise

from geometry_msgs.msg import PointStamped


class ObjectDetector:
    def __init__(self, topic_name):
        self.topic_name = topic_name

    def find_red_object(self):

        promise = Promise(
            lambda resolve, reject: resolve(rospy.wait_for_message(self.topic_name, PointStamped)))

        return promise
