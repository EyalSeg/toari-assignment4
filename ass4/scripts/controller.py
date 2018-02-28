#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction

from movement import Movement
from objectDetector import ObjectDetector
from arm import Arm
from poses import Poses
import std_srvs.srv
global controller

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

        self.arm = Arm()

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
        print("Moving Arm ...?")
        #target = self.poses.transform_point(target, 'base_footprint')                
        self.arm.set_pose(target)

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


    def look_down(self, toggle_on=True):
        rospy.wait_for_service('/look_down')
        try:
            look = rospy.ServiceProxy('/look_down', std_srvs.srv.SetBool)
            look(toggle_on)            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pick(self):
        rospy.wait_for_service('/pick_go')
        try:
            p = rospy.ServiceProxy('/pick_go', std_srvs.srv.Trigger)
            print("attempting picking")
            p()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
def menu():
    global controller
    menu_str = r"""Menu:
1) Navigate to Point
2) Move Arm
3) Look Up
4) Look Down
5) Pick
6) ipdb
Coose Action:
"""
    while(True):
        choice = input(menu_str)
        if choice == 1:
            point = input("Enter Coordinates: ")
            controller.move(list(point[:2]) + [0.], [0, 0, 0, 1.])
        elif choice == 2:
            point = input("Enter Coordinates: ")
            controller.move_arm(point) # (0.4, 0, 0.5) seems like a good one
        elif choice == 3:            
            controller.look_down(False)
        elif choice == 4:
            controller.look_down(True)
        elif choice == 5:
            controller.pick()
        elif choice == 6:
            import ipdb; ipdb.set_trace()
        elif choice == -1:
            return
            
if __name__ == '__main__':
    try:
        controller = Controller()
        menu()
        #controller.move_home2elevator()
        # controller.move_elevator2home()


        # controller.find_red_object()

        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
