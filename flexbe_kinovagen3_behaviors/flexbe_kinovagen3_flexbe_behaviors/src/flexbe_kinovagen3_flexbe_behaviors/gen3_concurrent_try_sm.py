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
from flexbe_kinovagen3_flexbe_states.move_to_8gripper_open_close import GripperControlState_open_close
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 08 2026
@author: amumu
'''
class gen3_concurrent_trySM(Behavior):
	'''
	Try running states simultinioulsy
	'''


	def __init__(self):
		super(gen3_concurrent_trySM, self).__init__()
		self.name = 'gen3_concurrent_try'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458, x:130 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:472, x:130 y:472, x:241 y:459, x:330 y:472, x:430 y:472, x:530 y:472
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('move_to_home', 'reached')]),
										('failed', [('move_to_home', 'failed')]),
										('finished', [('move_z_gripper', 'done')]),
										('failed', [('move_z_gripper', 'failed')])
										])

		with _sm_container_0:
			# x:172 y:52
			OperatableStateMachine.add('move_to_home',
										MoveToJointPositionState(joint_positions=[0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=5.0),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:423 y:179
			OperatableStateMachine.add('move_z_gripper',
										GripperControlState_open_close(open_position=0.0, close_position=0.7, effort=10.0, wait_time=0.5, repetitions=5),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:376 y:58
			OperatableStateMachine.add('move_to_zero',
										MoveToJointPositionState(joint_positions=[0, 0, 0, 0, 0, 0, 0], duration=5.0),
										transitions={'reached': 'Container', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:445 y:304
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
