#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from std_msgs.msg import Header

from promise import Promise

class Movement:
    def __init__(self, nav_server, map_frame):
        self.nav_server = nav_server

        self.map_frame = map_frame

    def move(self, coordinate, orientation):
        """ moves to the target destination.

        coordinate: an array containing [x, y]
        orientation: an array representing a quaternion
        """
        position = self.coord_to_position(coordinate, orientation)

        destination = MoveBaseGoal()
        destination.target_pose = position

        promise = Promise()

        self.nav_server.send_goal(destination, done_cb=lambda state, result: promise.do_resolve(True))
        return promise

    def coord_to_position(self, coordinate, orientation):
        header = Header()
        header.frame_id = self.map_frame

        pose = Pose()
        pose.position = Point(coordinate[0], coordinate[1], 0)
        pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])

        stamped = PoseStamped(header=header, pose=pose)
        return stamped


