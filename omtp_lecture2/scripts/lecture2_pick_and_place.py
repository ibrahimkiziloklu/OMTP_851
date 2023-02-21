#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list





def simple_pick_place():
    # Write a pick and place pipeline and execute with MoveIt and Gazebo
    # <<write your code here>>

    """
    initialize moveit_commander and rospy node
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_pick_place", anonymous=True)

    


    
    #########################################
    #MoveGroupCommander object: Interface to a planning group(group of joints).
    #Used to plan and execute motions.
    #########################################
    robot_arm_group = moveit_commander.MoveGroupCommander("panda_arm")
    robot_hand_group = moveit_commander.MoveGroupCommander("panda_tool")
    robot_arm_2_group = moveit_commander.MoveGroupCommander("panda_arm_2")
    robot_hand_2_group = moveit_commander.MoveGroupCommander("panda_tool_2")

    #########################################
    #Action clients to the ExecuteTrajectory action server .
    #########################################
    robot_arm_client = actionlib.SimpleActionClient(
        "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction)
    robot_arm_client.wait_for_server()
    rospy.loginfo("Execute Trajectory server is available for robot_arm")

    #########################################
    #Set a named joint config as a goal for move group.
    #Named joint config are the robot poses defined via MoveIt! Setup Assistant.
    #########################################
    robot_arm_group.set_named_target("robot_up")

    #Plan to the desired robot joint-space goal using the default planner (RRTConnect).
    robot_arm_plan_home = robot_arm_group.plan()
    #Goal message object for the action server
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    #Update the trajectory in the goal msg
    robot_arm_goal.trajectory = robot_arm_plan_home

    #Send the goal to the action server.
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    #########################################
    #Send the robot to another predefined pose
    #########################################
    robot_arm_group.set_named_target("robot_ready")
    robot_arm_plan_pregrasp = robot_arm_group.plan()
    #Goal message object for the action server
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    #Update the trajectory in the goal msg
    robot_arm_goal.trajectory = robot_arm_plan_pregrasp 
    #Send the goal to the action server.
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    #########################################
    #Cartesian Paths
    #########################################
    # You can plan a cartesian path directly by specifying a list of waypoints
    # For the end effector to go through.
    waypoints =[]
    #starts with the current pose
    current_pose = robot_arm_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = robot_arm_group.get_current_pose()

    #create linear offsets to the current pose
    new_eef_pose = geometry_msgs.msg.Pose()

    #Manual offsets bcz no camera yet.
    new_eef_pose.position.x = current_pose.pose.position.x +0.00
    new_eef_pose.position.y = current_pose.pose.position.x +0.00
    new_eef_pose.position.z = current_pose.pose.position.x -0.40

    # Retain orientation of the current pose.

    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)
    waypoints.append(new_eef_pose)
    waypoints.append(current_pose.pose)
    print(new_eef_pose.position)
    print(current_pose.pose.position)

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef step in cartesian
    ## translation. We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    fraction  = 0.0

    for count_cartesian_path in range(0,3):
        if fraction < 1.0:
            (plan_cartesian,fraction) = robot_arm_group.compute_cartesian_path(
                                        waypoints, #waypoints to follow
                                        0.01,      #eef_step
                                        0.0)       #jump_threshold
        else:
            break
        


    robot_arm_goal= moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = plan_cartesian
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    ########################
    # Move the robot back to its ready pose
    ########################
    robot_arm_group.set_named_target("robot_ready")
    robot_arm_plan_place = robot_arm_group.plan()
    robot_arm_goal= moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = robot_arm_plan_place
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    ########################
    # When finished shut down moveit_commander
    ########################
    moveit_commander.roscpp_shutdown()





if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass
