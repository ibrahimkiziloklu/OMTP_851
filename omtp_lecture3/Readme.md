# Object Detection and Grasping 

In this file we implemented the several cameras to detect the object and move towards the object using MoveIt motion planning framework in our factory enviroment.

## Dependencies 

    cmake_modules
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    geometry_msgs
    message_generation
    roscpp
    std_msgs
    std_srvs
    tf

## Logical Camera

Logical Camera has been integrated into the box_world.world file and that file has been called in the launch file. box_world includes camera and the several objects to detect and grab.

## Transform Object Pose

The script initializes a rospy node called 'transform_object_pose' and subscribes to the 'omtp/my_logical_camera1' topic, which outputs LogicalCameraImage messages.The logical_camera_callback function checks if the logical camera has detected a box object. If it has, the script initializes the movement commander for the robot arm and hand groups, and moves the robot arm to a predefined "robot_ready" position.

The detected box object's pose is transformed from the logical_camera1_frame coordinate frame to the world coordinate frame using a TransformListener. The script then computes a Cartesian path for the robot's end effector to follow towards the object. The robot arm executes the planned trajectory and moves towards the box object.

## 2d Camera 
For the real world application 2d camera has been integrated into the xacro file of the factory environment. The camera is placed on top of the conveyor belt and it is looking at the conveyor and the objects on top of it.The 2D camera is mounted on a fixed joint and provides a view of the workspace. The camera publishes images on the /camera1/image_raw topic and camera information on the /camera1/camera_info topic. 

## Initialization
Our pick and place script is written in python and it is located in the omtp_lecture3/scripts folder. The script is executed with the following command:

    $ rosrun omtp_lecture3 transform_object_pose.py

Before running the code one needs to start the roscore with the omtp factory:

    $ roslaunch omtp_lecture3 omtp_lecture3.launch
    

