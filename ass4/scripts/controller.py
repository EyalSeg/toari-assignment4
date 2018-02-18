#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

from movement import Movement
from objectDetector import ObjectDetector
from arm import Arm
from poses import Poses


class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        navigation.wait_for_server()

        self.poses = Poses()

        self.movement = Movement(navigation, "/map", self.poses)
        self.move = self.movement.move

        self.object_detector = ObjectDetector("/object_detection/coordinates")
        self.find_red_object = self.object_detector.find_red_object

        # self.arm = Arm()

        print("I'm up")

        # rospy.spin()

    def test_func(self, coordinates, offset):
        goal = self.movement.get_coordinate_near(coordinates, offset)
        print goal
        self.movement.move(desired_pose=goal)

    def move_to_object(self, location_stamped):
        loc = self.poses.transform_point(location_stamped, '/map')
        coordinate = [loc.point.x, loc.point.y]
        goal = self.movement.get_pose_near(coordinate, 0.5)

        print "moving to "
        print goal

        return self.movement.move(desired_pose=goal)

    def finished_moving(self):
        print "finished moving"

        rospy.signal_shutdown(0)
        return True

    def move_arm(self, target):
        print 1
        target = self.poses.transform_point(target, 'base_footprint')

        print target
        self.arm.set_pose(target.point)

if __name__ == '__main__':
    try:
        controller = Controller()

        controller.find_red_object() \
                .then(lambda location_stamped: controller.move_to_object(location_stamped))\
                .then(lambda result: controller.finished_moving())

        # controller.movement.move_to
        # controller.find_red_object()\
        #     .then(lambda location_stamped: controller.movement.move_to_object(location_stamped))\
        #     .then(lambda result: controller.finished_moving())

        #an aync navigation example. PROMISES ARE KEWL
        # controller.movement.move([0.9, -1.8], [0, 0, -0.7, 0.7])\
        #     .then(controller.move([-2.6, -3.2], [0, 0, 1, 0.01]))\
        #     .then(controller.move([-8.7, -1.5], [0, 0, 0.3, 0.9]))\
        # controller.find_red_object()\
        #     .then(lambda result: controller.move_to_object(result))\

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
