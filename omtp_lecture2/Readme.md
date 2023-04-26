# Manipulation with MoveIt! and Gazebo

This package contains the code for the second lecture of the course "Object Manipulation and Task Planning" (OMTP) at Aalborg University.We have used the MoveIt! framework to control the robot arm and hand in the Gazebo simulation environment.Environment has been created in the first lecture of the course.

## Dependencies

    moveit_ros_move_group
    moveit_fake_controller_manager
    moveit_kinematics
    moveit_planners_ompl
    moveit_ros_visualization
    moveit_setup_assistant
    moveit_simple_controller_manager
    joint_state_publisher
    joint_state_publisher_gui
    robot_state_publisher
    rviz
    tf2_ros
    xacro

## Movelt! Setup Assistant
MoveIt! Setup Assistant is a GUI for creating a 
MoveIt! configuration package for a robot. We will create the grasping motion planning easily with the help of this assistant.

• Launch MoveIt Setup Assistant

    $ roslaunch moveit_setup_assistant setup_assistant.launch



## Pose Configuration for Robot Arm , Hand 

In the setup assistant , the robot arm and hand has been configured with the following poses. These poses can be changed with the help of rviz joint state publisher.
    - Robot_Up Position
    - Robot_Ready Position
    - Robot_Grasp Position
    - Robot_Place Position


## PID Gains for Robot Arm and Hand

PID gains for each joint has been configured in the omtp_support/urdf/panda_arm_hand.xacro file.It can be changed with the help of rqt_reconfigure.Rqt helps us change the PID easily while changing the parameters we can see the robot arm visually.An example of the configuration is given below:

    $ rosrun rqt_reconfigure rqt_reconfigure

    panda_1_joint1:
      p: 12000
      d: 100
      i: 1
      i_clamp: 1

## MoveIt Commander

Planning can be executed in the rviz , or we can use the moveit commander to execute the planning in the python script.

## Pick and Place Script 

Our pick and place script is written in python and it is located in the omtp_lecture2/scripts folder. The script is executed with the following command:

    $ rosrun omtp_lecture2 pick_and_place.py

Before running the code one needs to start the roscore with the omtp factory:

    $ roslaunch omtp_lecture2 omtp_moveit_gazebo.launch
    
Also one thing to note is that one needs to hardcode the position of the banana since we do not have a sensor to tell the robot where it is.

