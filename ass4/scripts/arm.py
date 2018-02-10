#!/usr/bin/env python

import moveit_commander
from promise import Promise
from geometry_msgs.msg import Pose, Point


class Arm:
    def __init__(self):
        self.group = Arm.get_group()
        self.robot = moveit_commander.RobotCommander()

    def set_pose(self, point_stamped, orientation = None):
        print 'got here'
        group = self.group
        target_pose = Pose()

        if orientation is not None:
            target_pose.orientation = orientation
        else:
            target_pose.orientation.w = 1.

        target_pose.position = point_stamped
        try:
            group.set_pose_target(target_pose)
        except Exception as e:
            print e
        print 'planning'

        # plan = group.plan()
        # group.go()
        return Promise.resolve(group.plan())\
            .then(Promise.resolve(group.go(wait=True)))

    @staticmethod
    def get_group():
        group = moveit_commander.MoveGroupCommander("arm")
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.5)
        group.set_num_planning_attempts(500)
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_pose_reference_frame("base_footprint")
        group.set_goal_joint_tolerance(0.1)

        return group
