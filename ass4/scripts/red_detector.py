#!/usr/bin/env python

import rospy

import numpy as np
import cv2

from cv_bridge import CvBridge
from cv_helper import CvHelper

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, PointStamped, Point

import sensor_msgs.point_cloud2 as pc2



class RedDetector:

    def __init__(self):

        rospy.init_node('red_detector', anonymous=True)

        self.cv_bridge = CvBridge()
        self.frame_id = "kinect2_depth_optical_frame"

        rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.on_new_img)
        rospy.Subscriber("/kinect2/hd/points", PointCloud2, self.on_new_img)

        self.red_mask_publisher = rospy.Publisher("/kinect2/red_mask", Image, queue_size=1)
        self.object_coordinates_publisher = rospy.Publisher("/object_detection/coordinates", PointStamped, queue_size=1)
        self.object_view_publisher = rospy.Publisher('/object_detection/image_view', Image, queue_size=1)

        print "I'm up"
        rospy.spin()

    def on_new_img(self, msg):
        bgr = RedDetector.pc2_to_bgr(msg)

        reds = CvHelper.maskRed(bgr)
        self.red_mask_publisher.publish(self.cv_bridge.cv2_to_imgmsg(reds))

        contour = CvHelper.find_largest_contour(reds)
        if contour is None:
            self.object_view_publisher.publish(self.cv_bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))
            return

        try:
            center = CvHelper.find_contour_center(contour)
        except Exception:
            # TODO: fix the excpetion! See more details inside the function
            # swallowed the exception so the node will not crash.
            return

        cv2.circle(bgr, (center[0], center[1]), 30 ,(0, 255, 0), 5,  -1)
        self.object_view_publisher.publish(self.cv_bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))

        transform = RedDetector.get_transform(msg, center)
        self.publish_object(transform)

    def publish_object(self, transform):
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = rospy.get_rostime()

        point = Point(transform[0], transform[1], transform[2])
        stamped = PointStamped(header, point)

        self.object_coordinates_publisher.publish(stamped)

    @staticmethod
    def pc2_to_bgr(pc):
        x = np.frombuffer(pc.data, 'uint8').reshape(-1, 8, 4)
        bgr = x[:pc.height*pc.width, 4, :3].reshape(pc.height, pc.width, 3)

        return np.array(bgr)

    @staticmethod
    def get_transform(cloud, point):
        coordiante = [int(point[0]), int(point[1])]
        generator = pc2.read_points(cloud, field_names=['x', 'y', 'z'], uvs=[coordiante])

        return generator.next()

if __name__ == '__main__':
    try:
        RedDetector = RedDetector()
    except rospy.ROSInterruptException:
        pass
