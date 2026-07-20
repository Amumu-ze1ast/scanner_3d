#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_kinovagen3_flexbe_states.move_to_1joint_position_state import MoveToJointPositionState
from flexbe_kinovagen3_flexbe_states.move_to_6Straight_path import MoveIn_auto_StraightPath
from flexbe_kinovagen3_flexbe_states.move_to_7gripoper import GripperControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 06 2026
@author: amumu
'''
class Gen3_check123SM(Behavior):
	'''
	Check the straight line code and gripper code
	'''


	def __init__(self):
		super(Gen3_check123SM, self).__init__()
		self.name = 'Gen3_check123'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:142 y:487, x:63 y:222
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:350 y:25
			OperatableStateMachine.add('move_to_zero',
										MoveToJointPositionState(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=5.0),
										transitions={'reached': 'move_to_home', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:350 y:833
			OperatableStateMachine.add('gripper_open',
										GripperControlState(position=0.0, effort=10.0, wait_time=1.0),
										transitions={'done': 'move_to_home2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:347 y:231
			OperatableStateMachine.add('move_in_straight_path',
										MoveIn_auto_StraightPath(end_x=0.772, end_y=0.003, end_z=0.478, num_waypoints=5, roll=0.0, pitch=1.57, yaw=0.0, duration=5),
										transitions={'reached': 'gripper_close', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:362 y:544
			OperatableStateMachine.add('move_in_straight_path2',
										MoveIn_auto_StraightPath(end_x=0.456, end_y=0.002, end_z=0.520, num_waypoints=5, roll=0.0, pitch=1.57, yaw=0.0, duration=3),
										transitions={'reached': 'move_to_basket', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:348 y:445
			OperatableStateMachine.add('move_in_straight_up',
										MoveIn_auto_StraightPath(end_x=0.772, end_y=0.003, end_z=0.520, num_waypoints=3, roll=0.0, pitch=1.57, yaw=0.0, duration=2),
										transitions={'reached': 'move_in_straight_path2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:627 y:790
			OperatableStateMachine.add('move_to basket3',
										MoveIn_auto_StraightPath(end_x=0.209, end_y=-0.406, end_z=0.234, num_waypoints=2, roll=1.571, pitch=-0.001, yaw=0.472, duration=2),
										transitions={'reached': 'gripper_open', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:362 y:656
			OperatableStateMachine.add('move_to_basket',
										MoveToJointPositionState(joint_positions=[0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=2),
										transitions={'reached': 'move_to_basket2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:636 y:570
			OperatableStateMachine.add('move_to_basket2',
										MoveToJointPositionState(joint_positions=[1.1, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=2),
										transitions={'reached': 'move_to basket3', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:352 y:127
			OperatableStateMachine.add('move_to_home',
										MoveToJointPositionState(joint_positions=[0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=5.0),
										transitions={'reached': 'move_in_straight_path', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:46 y:683
			OperatableStateMachine.add('move_to_home2',
										MoveToJointPositionState(joint_positions=[0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=3.0),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:352 y:345
			OperatableStateMachine.add('gripper_close',
										GripperControlState(position=0.6, effort=10.0, wait_time=1.0),
										transitions={'done': 'move_in_straight_up', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
