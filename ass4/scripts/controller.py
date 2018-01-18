#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from std_msgs.msg import Header

class Controller:
    def __init__(self):

        rospy.init_node('talker', anonymous=True)

        self.navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.navigation.wait_for_server()
        print("I'm up")


        #rospy.spin()

    def finished_moving(self, state, result):
        print "finished moving"

        rospy.signal_shutdown(0)

    def move(self, x, y, z = 0):
        print "moving!"

        header = Header()
        header.frame_id = "/map"

        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = Quaternion(0, 0, 1, 0.3)

        stamped = PoseStamped(header=header, pose=pose)

        goal = MoveBaseGoal()
        goal.target_pose = stamped

        self.navigation.send_goal(goal, done_cb=self.finished_moving)

        rospy.spin()

if __name__ == '__main__':
    try:
        controller = Controller()
        controller.move(-8, -3)
    except rospy.ROSInterruptException:
        pass