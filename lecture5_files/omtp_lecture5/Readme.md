# CNN in Practical Robotic Applications

This repository contains the code for the lecture 5 of the course . We will use the 2d camera to for detection and apply the yolov3 algorithm to detect the objects on the conveyor belt. The 2d camera is mounted on a fixed joint and provides a view of the workspace. The camera publishes images on the /camera1/image_raw topic and camera information on the /camera1/camera_info topic.

## Prerequisites
The prequisites for this package is inside the [requirements.txt](requirements.txt) file. To install them, navigate to the root of the package and r:
```
$ sudo pip install -r requirements.txt
```

    PyTorch 
    OpenCV
    torchvision
    skimage
    NumPy
    SciPy
    rospkg
    Gazebo
    MoveIt!
    xacro
    joint_state_publisher
    robot_state_publisher
    topic_tools

## Installation
Navigate to your catkin workspace and run:
```
$ catkin build omtp_lecture5
$ catkin build yolov3_pytorch_ros
```
## Configuration
1. First, make sure to put your weights in the [models](models) folder. For the **training process** in order to use custom objects, please refer to the original [YOLO page](https://pjreddie.com/darknet/yolo/). As an example, to download pre-trained weights from the COCO data set, go into the [models](models) folder and run:
```
wget http://pjreddie.com/media/files/yolov3.weights
```

2. Modify the parameters in the [launch file](launch/detector.launch) and launch it. You will need to change the `image_topic` parameter to match your camera, and the `weights_name`, `config_name` and `classes_name` parameters depending on what you are trying to do.

## Factory Launch File 

The provided launch file omtp_lecture5.launch sets up the following components:
Gazebo world: Loads the yolo_world.world file containing the simulation environment.
Robot description: Loads the robot's URDF (Universal Robot Description Format) file omtp_panda5.urdf.xacro.


    $ roslaunch omtp_lecture5 omtp_lecture5.launch



## Start yolov3_pytorch_ros node

Make sure that detector.py is executable by navigating to the src folder and typing

    $ chmod +x detector.py


then run the node with

    $ roslaunch yolov3_pytorch_ros detector.launch

The node will subscribe to the specified image topic, process the images using the YOLOv3 model, and publish the detected bounding boxes and visualization of the detections.

## Usage

The node subscribes to an image topic (default: /camera/rgb/image_raw) and listens for incoming image messages.

When an image message is received, the imageCb function is called to process the image, detect objects using the YOLOv3 model, and publish the detected bounding boxes and visualization.

The detected bounding boxes are published as BoundingBoxes messages to the specified topic (default: /detected_objects).

The visualization of the detections is published as an Image message to the specified topic (default: /detections_image) if the publish_image parameter is set to True.