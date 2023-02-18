#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


def simple_pick_place():
    # Write a pick and place pipeline and execute with MoveIt and Gazebo
    # <<write your code here>>
    pass


if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass
