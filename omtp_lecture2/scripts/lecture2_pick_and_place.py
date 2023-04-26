#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs


def simple_pick_place():
    # initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("simple_pick_place", anonymous = True)

    # groups
    robot_arm_group = moveit_commander.MoveGroupCommander("robot_arm")
    robot_hand_group = moveit_commander.MoveGroupCommander("robot_hand")


    # action-client
    robot_arm_client = actionlib.SimpleActionClient(
        'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    robot_arm_client.wait_for_server()
    rospy.loginfo("Execute trajectory server is available for robot_arm")

    robot_hand_client = actionlib.SimpleActionClient(
        'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    robot_hand_client.wait_for_server()
    rospy.loginfo("Execute trajectory server is available for robot_arm")


    # robot_arm_group.set_named_target("robot_up")

    # plan_success, robot_arm_plan_home, planning_time, error_code = robot_arm_group.plan()
    # robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    # robot_arm_goal.trajectory = robot_arm_plan_home

    # robot_arm_client.send_goal(robot_arm_goal)
    # robot_arm_client.wait_for_result()

    robot_arm_group.set_named_target("robot_ready")
    
    plan_success, robot_arm_plan_pregrasp, planning_time, error_code = robot_arm_group.plan()
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = robot_arm_plan_pregrasp
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    robot_hand_group.set_named_target("tool_open")
    plan_success, robot_hand_plan_place, planning_time, error_code = robot_hand_group.plan()
    robot_hand_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_hand_goal.trajectory = robot_hand_plan_place
    robot_hand_client.send_goal(robot_hand_goal)
    robot_hand_client.wait_for_result()

    # cartesian coordinates

    waypoints = []

    current_pose = robot_arm_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = robot_arm_group.get_current_pose()

    #box positions
    x = 0.081422
    y = 11.151620
    z = 1.002484

    diff_x = current_pose.pose.position.x - x
    diff_y = current_pose.pose.position.y - y
    diff_z = current_pose.pose.position.z - z


    new_eef_pose = geometry_msgs.msg.Pose()
    new_eef_pose.position.x = current_pose.pose.position.x - diff_x
    new_eef_pose.position.y = current_pose.pose.position.y - diff_y
    new_eef_pose.position.z = current_pose.pose.position.z - diff_z + 0.3

    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)
    waypoints.append(new_eef_pose)

    fraction = 0.0
    for count_cartesian_path in range(3):
        if fraction < 1.0:
            plan_cartesian, fraction = robot_arm_group.compute_cartesian_path(
                waypoints,
                0.01, # eef_step
                0.0   #jump threshold
            )
        else:
            break
    
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = plan_cartesian
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    new_eef_pose.position.x = current_pose.pose.position.x - diff_x
    new_eef_pose.position.y = current_pose.pose.position.y - diff_y
    new_eef_pose.position.z = current_pose.pose.position.z - diff_z + 0.19

    new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)
    waypoints.append(new_eef_pose)

    fraction = 0.0
    for count_cartesian_path in range(3):
        if fraction < 1.0:
            plan_cartesian, fraction = robot_arm_group.compute_cartesian_path(
                waypoints,
                0.01, # eef_step
                0.0   #jump threshold
            )
        else:
            break
    
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = plan_cartesian
    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    robot_hand_group.set_named_target("tool_closed")
    plan_success, robot_hand_plan_place1, planning_time, error_code = robot_hand_group.plan()
    robot_hand_goal1 = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_hand_goal1.trajectory = robot_hand_plan_place1
    robot_hand_client.send_goal(robot_hand_goal1)
    rospy.sleep(0.5)
    robot_hand_client.wait_for_result()


    robot_arm_group.set_named_target("robot_ready")
    plan_success, robot_arm_plan_place, planning_time, error_code = robot_arm_group.plan()
    robot_arm_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot_arm_goal.trajectory = robot_arm_plan_place

    robot_arm_client.send_goal(robot_arm_goal)
    robot_arm_client.wait_for_result()

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        simple_pick_place()
    except rospy.ROSInterruptException:
        pass