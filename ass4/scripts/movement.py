#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from std_msgs.msg import Header

import tf.transformations as transformations


from promise import Promise
from poses import Poses

import numpy
import math

class Movement:
    def __init__(self, nav_server, map_frame, poses_service):
        self.nav_server = nav_server
        self.map_frame = map_frame

        self.poses = poses_service

    def move(self, coordinate=None, orientation=None, desired_pose=None):
        """ moves to the target destination.
        pass either coordinate & orientation or a desired_pose that contains both
        coordinate: an array containing [x, y]
        orientation: an array representing a quaternion
        """

        if desired_pose is None:
            desired_pose = self.coord_to_position(coordinate, orientation)

        destination = MoveBaseGoal()
        destination.target_pose = desired_pose

        promise = Promise()

        self.nav_server.send_goal(destination, done_cb=lambda state, result: promise.do_resolve(True))
        return promise


    def get_coordinate_near(self, coordinate, offset):
        current_pose = self.poses.get_current_pose()
        current_position = Movement.position2nparray(current_pose.pose.position)

        diff = numpy.array(coordinate) - current_position
        away = offset / numpy.linalg.norm(diff)
      #  away = offset / math.sqrt(diff[0]*diff[0] + diff[1]*diff[1]) #shamelessly copied from robotican's drive2object

        destination_pos = current_position + diff * (1 - away)

        yaw = math.atan2(diff[1], diff[0])
        orientation = transformations.quaternion_from_euler(0, 0, yaw)

        return self.coord_to_position(destination_pos, orientation)



    def coord_to_position(self, coordinate, orientation):
        header = Header()
        header.frame_id = self.map_frame

        pose = Pose()
        pose.position = Point(coordinate[0], coordinate[1], 0)
        pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

        stamped = PoseStamped(header=header, pose=pose)
        return stamped

    @staticmethod
    def position2nparray(position):
        return numpy.array([position[0], position[1]])
