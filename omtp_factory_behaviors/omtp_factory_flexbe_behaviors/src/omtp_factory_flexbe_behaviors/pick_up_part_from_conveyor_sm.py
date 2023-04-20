#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState
from omtp_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from omtp_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
from omtp_factory_flexbe_states.tool import Tool
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Mar 15 2023
@author: Kutay Shentyurk
'''
class pickuppartfromconveyorSM(Behavior):
	'''
	an hello work pick and move to home1
	'''


	def __init__(self):
		super(pickuppartfromconveyorSM, self).__init__()
		self.name = 'pick up part from conveyor'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'robot_arm'
		home1 = [0, 0, 0, -1.57, 0, 1.57, 0]
		robot1_joint_names = ['panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7']
		robot1_tool_link = 'panda_1_link8'
		tool_closed = [0.0, 0.0]
		tool_open = [0.04, 0.04]
		tool_name = 'robot_hand'
		tool_joints = ['panda_1_finger_joint1', 'panda_1_finger_joint2']
		# x:1116 y:495, x:78 y:626
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = home1
		_state_machine.userdata.joint_names = []
		_state_machine.userdata.object_pose = []
		_state_machine.userdata.home = home1
		_state_machine.userdata.names = robot1_joint_names

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:74 y:56
			OperatableStateMachine.add('open',
										Tool(move_group=tool_name, joint_name=tool_joints, tool_goal=tool_open, action_topic='/move_group'),
										transitions={'reached': 'detect', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off})

			# x:776 y:288
			OperatableStateMachine.add('close',
										Tool(move_group=tool_name, joint_name=tool_joints, tool_goal=tool_closed, action_topic='/move_group'),
										transitions={'reached': 'move home', 'planning_failed': 'failed', 'control_failed': 'move home'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off})

			# x:283 y:94
			OperatableStateMachine.add('detect',
										DetectPartCameraState(ref_frame='robot1_pedestal_link', camera_topic='/omtp/my_logical_camera1', camera_frame='logical_camera1_frame'),
										transitions={'continue': 'calculate position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose'})

			# x:870 y:419
			OperatableStateMachine.add('move home',
										MoveitToJointsDynState(move_group=pick_group, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'home', 'joint_names': 'names'})

			# x:646 y:195
			OperatableStateMachine.add('move to grasp',
										MoveitToJointsDynState(move_group=pick_group, action_topic='/move_group'),
										transitions={'reached': 'close', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:467 y:141
			OperatableStateMachine.add('calculate position',
										ComputeGraspState(group='robot_arm', offset=0.1, joint_names=robot1_joint_names, tool_link=robot1_tool_link, rotation=3.1415),
										transitions={'continue': 'move to grasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose', 'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
