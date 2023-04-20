# Building a Robot Simulation Environment #


Design of the factory has been made in the omtp_factory.xacro. 
Map of AAU Manufacturing department (Fib 14 )has been added to the main xacro as fib_14 parent and linked to the world_interface .This Xacro file describes the robot setup and environment. It includes the following components:

    Fib14 Building
    Panda Robots (dual-arm setup)
    Robot Pedestals
    Bins
    Festo Modules

- A Fib14 building model imported from aau_lab_ros_models/urdf/fib14/fib14.urdf.xacro.
- One Panda robot from the franka_description package, with  arm and a gripper. The models are imported from franka_description/robots/panda_arm.xacro and franka_description/robots/hand.xacro.
- Pedestals for the robots, imported from omtp_support/urdf/robot_pedestal/robot_pedestal.urdf.xacro.
Three bin models, imported from omtp_support/urdf/bin/bin.urdf.xacro.

## Dependencies 

- joint_state_publisher
- robot_state_publisher
- rviz
- xacro
- franka_description  

 
## Usage

''' sh
$ roslaunch omtp_support visualize_omtp_factory.launch
'''




## Robot Manipulator, gripper and pedestal

From the franka _description folder, Franka robot with 7 DOF has been added with an hand tool attachment.
The robot is added to the main xacro file connected to the pedestal link.

For the end effector, a gripper has been added to the robot. The Gripper is added to the main xacro file connected to the end effector link of the robot.Different kind of grippers can be added to the robot by changing the xacro file. Vacuum example is below:


''' xacro
<!-- Franka arm and hand XACRO model -->
<xacro:include filename="$(find franka_description)/robots/panda_arm.xacro"/>
<xacro:include filename="$(find franka_description)/robots/hand.xacro"/>
<!-- Vacuum Gripper1 -->
<xacro:include filename="$(find hrwros_support)/urdf/vacuum_gripper/vacuum_gripper.urdf.xacro"/>
<xacro:vacuum_gripper_urdf prefix="vacuum_gripper1_" joint_prefix="vacuum_gripper1_joint"/>
'''

## Enviromental Models

Conveyor belt modules has been placed similar to the real factory environment in the AAU Manufacturing department (Fib 14 ).They include bypass, robot, straight, T modules and bins. These modules are included from the aau_lab_ros_models package and are set up with specified positions in the environment.

- Five Festo module models: festo_bypass_1, festo_robot_1, festo_straight_1, festo_straight_4, festo_straight_2, festo_straight_3 and festo_t_1. 
- These are imported from aau_lab_ros_models.

