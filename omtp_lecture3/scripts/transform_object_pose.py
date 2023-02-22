#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# TU Delft Robotics Institute.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Delft University of Technology nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Modified by: Mukunda Bharatheesha
##
import rospy
from omtp_lecture3.msg import LogicalCameraImage
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs


def logical_camera_callback(data):
    # Check if the logical camera has seen our box which has the name 'object'.
    for gazebo_object in data.models:
        if (gazebo_object.type == 'box'):
            # Create a pose stamped message type from the camera image topic.
            object_pose = geometry_msgs.msg.PoseStamped()
            # Only works with a timestamp, timing information is very important in tf
            object_pose.header.stamp = rospy.Time.now()
            # The logical camera always outputs the pose of an object it its own
            # reference frame and here it is called logical_camera1_frame
            # If we have more than one logical camera, this name will change
            object_pose.header.frame_id = "logical_camera1_frame"
            object_pose.pose.position.x = gazebo_object.pose.position.x
            object_pose.pose.position.y = gazebo_object.pose.position.y
            object_pose.pose.position.z = gazebo_object.pose.position.z
            object_pose.pose.orientation.x = gazebo_object.pose.orientation.x
            object_pose.pose.orientation.y = gazebo_object.pose.orientation.y
            object_pose.pose.orientation.z = gazebo_object.pose.orientation.z
            object_pose.pose.orientation.w = gazebo_object.pose.orientation.w
            while True:
                try:
                    object_world_pose = tf_buffer.transform(object_pose, "world")
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
            print('========================================')
            print('Pose of the object in the world reference frame is: %s', object_world_pose)
            print('========================================')
            print('Pose of the object in the logical camera reference frame is: %s', object_pose)
            print('========================================')
            print('Successfully transformed pose.')
            rospy.signal_shutdown('Successfully transformed pose.')
            print('========================================')
        else:
            # Do nothing.
            print('')


if __name__ == '__main__':
    print('========================================')
    # Initialize ROS node to transform object pose.
    print("Initializing node ...")
    rospy.init_node('transform_object_pose', anonymous=True)

    # Create a TF buffer in the global scope
    print("Creating tf buffer ...")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Subscribe to the logical camera topic.
    print("Subscribing to logical camera ...")
    rospy.Subscriber('omtp/my_logical_camera1',
                     LogicalCameraImage, logical_camera_callback)

    rospy.spin()
