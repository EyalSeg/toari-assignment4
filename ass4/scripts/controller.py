#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

from movement import Movement
from objectDetector import ObjectDetector

class Controller:
    def __init__(self):
        rospy.init_node('talker', anonymous=True)

        navigation = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        navigation.wait_for_server()

        self.movement = Movement(navigation, "/map")
        self.move = self.movement.move

        self.object_detector = ObjectDetector("/object_detection/coordinates")
        self.find_red_object = self.object_detector.find_red_object

        print("I'm up")

        # rospy.spin()

    def finished_moving(self):
        print "finished moving"

        rospy.signal_shutdown(0)
        return True


if __name__ == '__main__':
    try:
        controller = Controller()

        #an aync navigation example. PROMISES ARE KEWL
        # controller.move([0.9, -1.8], [0, 0, -0.7, 0.7])\
        #     .then(lambda result: controller.move([-2.6, -3.2], [0, 0, 1, 0.01]))\
        #     .then(lambda result: controller.move([-8.7, -1.5], [0, 0, 0.3, 0.9]))\
        #     .then(lambda result: controller.move([-8.7, 3], [0, 0, -1, 0.01]))\
        #     .then(lambda result: controller.finished_moving())

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
