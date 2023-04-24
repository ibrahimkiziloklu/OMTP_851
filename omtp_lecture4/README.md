# ROBOT BEHAVIOR DESIGN WITH STATE MACHINES USING FLEXBE APP #

Via flexBE app we can create machines that does multiple tasks in a sequence without the need to hardcode them. Normally, This task is overcome using subscriber publisher nodes, and action service topics.

## Dependencies
```sh
catkin
genmsg
genpy
cpp_common
rostime
roscpp_traits
roscpp_serialization
message_runtime
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
ros_environment
rospack
roslib
rospy
```
## Other Modules Needed

You need to include the modules:
```sh
- flexbe_app
- flexbe_behavior_engine
- generic_flexbe_states
- omtp_factory_behaviors
- panda_v1
- aau_lab_ros_modules
- franka_description
- omtp_support
```
in your catkin workspace to be able to run the module. Remember to catkin build or catkin_make after adding the modules. Otherwise, flexbe app wont recognise the behaviors.

## Running the module
---
First one needs to start an environment in gazebo. We will be using the OMTP factory developed in omtp_lecture1 module, yet the urdf file can be found inside the urdf file of this module:
```sh
$ roslaunch omtp_lecture5 lecture5.launch
```
Next, one needs to launch FlexBE app.
```sh
$ roslaunch flexbe_app flexbe_full.launch
```
Realize that we are starting the flexbe full which means it will try to connect to a ros master. You can make changes to the behavior in this app or can also use the offline mod of flexbe, which is run with 
```sh
$ rosrun flexbe_app run_app
```
In any case, you need to choose the behavior developed. When in the flexbe app, you need to click on load behavior, which is in the middle of the menu at the top of the app window. The behavior we developed is called "pick up part from conveyor". Afterwards, you can run the app from runtime control.

## Defined Private Configurations and State Machine Userdata

The private config are constant data, and it is used to define certain robot poses, robot joint names, and joint groups that are defined in .srdf file in panda_v1. State machine userdata is used as input outputs for the states, and in this module, some of them are predefined and some are left blank. The blank ones are assigned values after states.

## Already built states

The module utilizes states that are already built in flexbe manipulation states that utilize moveit to drive the robot to a desired position. The used state is MoveitToJointDynState, which takes joint_values and joint_names. It is used in this module to drive the robot to the pick up posiiton and home config. There are also states provided by the lecturer, which are the DetectPartCameraState, and ComputeGraspState. They find the position of the object to be picked and calculate the state values to have the eneffector at that point, respectively.

## Tool state

The custom state that we designed is called Tool. It is a derivative of MoveitToJointDynState. The outcomes are the same. Yet, this state does not take inputs as userdata. Rather, it uses private config because the states of the grasping tool is predefined as close and open. 

## Implementation

The custom state is used in two different places in the statemachine. It is used to open the end effector and later when the end effector moves to the object, it is used to close it. When closing since the predefined values cannot be reached the control failed outcome of the state is attached to the next state rather than having it connect to the overall fail outcome since we do not have any other sensor to recognize if the grasp is succesfull or not.

## Who do I talk to? ##
* Kutay Shentyurk ([kshent22@student.aau.dk](mailto:kshent22@student.aau.dk))
