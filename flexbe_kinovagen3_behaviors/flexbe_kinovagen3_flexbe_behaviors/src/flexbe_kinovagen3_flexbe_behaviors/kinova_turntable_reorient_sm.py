#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from kinovagen3_flexbe_flexbe_states.move_to_9cartesian_offest import Gen3_MoveToCartesianPoseState_offset
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Feb 26 2026
@author: amumu
'''
class kinova_turntable_reorientSM(Behavior):
	'''
	kinova_turntable_reorient
	'''


	def __init__(self):
		super(kinova_turntable_reorientSM, self).__init__()
		self.name = 'kinova_turntable_reorient'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:395 y:174
			OperatableStateMachine.add('gen3_reorient_object',
										Gen3_MoveToCartesianPoseState_offset(x_offset=0.0, y_offset=0, z_offset=0.02, roll=0, pitch=2.2, yaw=0, duration=2),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
