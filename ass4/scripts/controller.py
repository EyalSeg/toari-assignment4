#!/usr/bin/env python

import rospy

def init():
    rospy.init_node('talker', anonymous=True)

    print("I'm up")

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass