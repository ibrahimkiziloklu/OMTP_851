#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from omtp_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from omtp_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 23 2022
@author: Simon
'''
class PickpartSM(Behavior):
	'''
	New behavior
	'''


	def __init__(self):
		super(PickpartSM, self).__init__()
		self.name = 'Pick part'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'robot1'
		home1 = [0,0,0,0,0,0,0]
		# x:478 y:248, x:98 y:259
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.object_pose = []
		_state_machine.userdata.pick_configuration = home1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Detect part',
										DetectPartCameraState(ref_frame='robot1_base_link', camera_topic='/omtp/logical_camera', camera_frame='logical_camera_frame'),
										transitions={'continue': 'Compute grasp configuration', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose'})

			# x:236 y:154
			OperatableStateMachine.add('Compute grasp configuration',
										ComputeGraspState(group=pick_group, offset=0.25, joint_names=['joint1'], tool_link='gripper_link8', rotation=0.0),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'object_pose', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
