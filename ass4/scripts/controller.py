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

    def move_home2elevator(self):
        self.move([0.410733197542, -0.376687458057, 0.0], [0,0, 0.855162922773, 0.518359311206])
        rospy.loginfo('goal reached: behind the door')

        self.move([-0.655349778083, 0.867898810977, 0.0], [0, 0, 0.905126074062, 0.42514325827])
        rospy.loginfo('goal reached: after the door')

        self.move([12.3223917556, 9.50084713397, 0.0], [0, 0, -0.281058133254, 0.95969074484])
        rospy.loginfo('goal reached: end of the hallway')

        # self.move([15.1777017985, 8.17533488251, 0.0], [0, 0, 0.0460119659733, 0.998940888635])
        # rospy.loginfo('goal reached: elevetor hallway')

        self.move([20.0990322997, 11.4698885036, 0.0], [0, 0, 0.870491573806, 0.492183319438])
        rospy.loginfo('goal reached: elevetor button')



    def move_elevator2home(self):
        self.move([12.7013056685, 9.79194745793, 0.0], [0, 0, -0.997673707286, 0.0681701825562])
        rospy.loginfo('goal reached: lab hallway')

        self.move([0.012765523603, -0.00185758867047, 0.0], [0, 0, 0, 1])
        rospy.loginfo('goal reached: lab door')

        self.move([0.211834775807, -0.43038693606, 0.0], [0, 0, -0.290002707577, 0.957025824938])
        rospy.loginfo('goal reached: lab ')

        self.move([4.60962563787, 0.0647232007319, 0.0], [0, 0, 0.973753551512, 0.227604966813])
        rospy.loginfo('goal reached: desk ')


if __name__ == '__main__':
    try:
        controller = Controller()

        #controller.move_home2elevator()
        controller.move_elevator2home()


        # controller.find_red_object() \
        #         .then(lambda location_stamped: controller.move_to_object(location_stamped))\
        #         .then(lambda result: controller.finished_moving())

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
