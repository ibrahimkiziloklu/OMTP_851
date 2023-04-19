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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Mar 19 2021
@author: Simon Bogh
'''
class MoverobottotwoposesSM(Behavior):
	'''
	This behavior will move the Franka robot to two poses
	'''


	def __init__(self):
		super(MoverobottotwoposesSM, self).__init__()
		self.name = 'Move robot to two poses'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		robot_arm_group = 'panda_arm'
		home_pos = [-0.048,-0.2,0.048,-1.6454,0.2401,1.6905,-0.08]
		robot_joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
		ready_pos = [-0.048,0.1,0.048,-1.6454,0.2401,1.6905,-0.08]
		# x:724 y:279, x:156 y:296
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.home_configuration = home_pos
		_state_machine.userdata.joint_names = robot_joint_names
		_state_machine.userdata.pick_configuration = ready_pos

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:184 y:107
			OperatableStateMachine.add('Move robot home',
										MoveitToJointsDynState(move_group=robot_arm_group, action_topic='/move_group'),
										transitions={'reached': 'Move robot to grasp', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'home_configuration', 'joint_names': 'joint_names'})

			# x:453 y:187
			OperatableStateMachine.add('Move robot to grasp',
										MoveitToJointsDynState(move_group=robot_arm_group, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
