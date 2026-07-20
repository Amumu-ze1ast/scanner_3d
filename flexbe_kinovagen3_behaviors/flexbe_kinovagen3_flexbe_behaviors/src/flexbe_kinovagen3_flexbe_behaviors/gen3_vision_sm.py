#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_kinovagen3_flexbe_states.move_to_10object_detection_save import DetectObjectSimpleState_save
from flexbe_kinovagen3_flexbe_states.move_to_12move_to_detected_straight import MoveToDetectedPositionState_straight
from flexbe_kinovagen3_flexbe_states.move_to_13straight_line_1_dxn import MoveIn_auto_StraightPath_1dxn
from flexbe_kinovagen3_flexbe_states.move_to_14cartesian_offset import MoveToCartesianPoseState_offset
from flexbe_kinovagen3_flexbe_states.move_to_1joint_position_state import MoveToJointPositionState
from flexbe_kinovagen3_flexbe_states.move_to_2cartesian_pose_state import MoveToCartesianPoseState
from flexbe_kinovagen3_flexbe_states.move_to_6Straight_path import MoveIn_auto_StraightPath
from flexbe_kinovagen3_flexbe_states.move_to_7gripoper import GripperControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 08 2026
@author: amumu
'''
class Gen3_visionSM(Behavior):
	'''
	vision
	'''


	def __init__(self):
		super(Gen3_visionSM, self).__init__()
		self.name = 'Gen3_vision'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458, x:68 y:253
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:130 y:458, x:230 y:458, x:330 y:458
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['detected_x', 'detected_y', 'detected_z'], conditions=[
										('finished', [('move_to_scan2', 'reached'), ('detect', 'detected')]),
										('failed', [('move_to_scan2', 'failed'), ('detect', 'failed')])
										])

		with _sm_container_0:
			# x:117 y:158
			OperatableStateMachine.add('move_to_scan2',
										MoveIn_auto_StraightPath(end_x=0.456, end_y=-0.202, end_z=0.434, num_waypoints=5, roll=0.0, pitch=1.57, yaw=0.0, duration=5),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:463 y:160
			OperatableStateMachine.add('detect',
										DetectObjectSimpleState_save(base_frame="base_link", min_area=800, timeout=5.0),
										transitions={'detected': 'finished', 'failed': 'failed'},
										autonomy={'detected': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detected_x': 'detected_x', 'detected_y': 'detected_y', 'detected_z': 'detected_z'})



		with _state_machine:
			# x:175 y:46
			OperatableStateMachine.add('move_to_scan1',
										MoveToCartesianPoseState(x=0.456, y=0.202, z=0.434, roll=0.0, pitch=1.57, yaw=0.0, duration=5.0),
										transitions={'reached': 'Container', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:412 y:765
			OperatableStateMachine.add('cartesian_offset',
										MoveToCartesianPoseState_offset(x_offset=0.2, y_offset=0.2, z_offset=0.2, roll=0.0, pitch=1.57, yaw=0.0, duration=3.0),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:795 y:74
			OperatableStateMachine.add('gripper_close',
										GripperControlState(position=0.6, effort=10.0, wait_time=1.0),
										transitions={'done': 'move_x_dxn_only', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1091 y:746
			OperatableStateMachine.add('gripper_open',
										GripperControlState(position=0.0, effort=10.0, wait_time=1.0),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:300 y:497
			OperatableStateMachine.add('just_detect',
										DetectObjectSimpleState_save(base_frame="base_link", min_area=800, timeout=5.0),
										transitions={'detected': 'move_to_detect1_exact', 'failed': 'failed'},
										autonomy={'detected': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detected_x': 'detected_x', 'detected_y': 'detected_y', 'detected_z': 'detected_z'})

			# x:53 y:764
			OperatableStateMachine.add('move_one_dxn',
										MoveIn_auto_StraightPath_1dxn(x_off_set=-0.2, y_off_set=0, z_off_set=0, num_waypoints=5, roll=0.0, pitch=1.57, yaw=0.0, duration=3),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:980 y:559
			OperatableStateMachine.add('move_to_basket',
										MoveToCartesianPoseState_offset(x_offset=-0.27, y_offset=-0.57, z_offset=-0.1, roll=0.0, pitch=3.14, yaw=0.0, duration=5.0),
										transitions={'reached': 'gripper_open', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1086 y:446
			OperatableStateMachine.add('move_to_default',
										MoveToJointPositionState(joint_positions=[0.0, 0.2600, 3.140, -2.2699, 0.0, 0.9599, 1.5700], duration=3),
										transitions={'reached': 'move_to_basket', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:307 y:339
			OperatableStateMachine.add('move_to_detect1',
										MoveToDetectedPositionState_straight(num_waypoints=5, roll=0.0, pitch=1.57, yaw=0.0, duration=3, x_offset=0.3),
										transitions={'reached': 'just_detect', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detected_x': 'detected_x', 'detected_y': 'detected_y', 'detected_z': 'detected_z'})

			# x:287 y:648
			OperatableStateMachine.add('move_to_detect1_exact',
										MoveToDetectedPositionState_straight(num_waypoints=2, roll=0.0, pitch=1.57, yaw=0.0, duration=2, x_offset=0.085),
										transitions={'reached': 'gripper_close', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detected_x': 'detected_x', 'detected_y': 'detected_y', 'detected_z': 'detected_z'})

			# x:807 y:220
			OperatableStateMachine.add('move_x_dxn_only',
										MoveIn_auto_StraightPath_1dxn(x_off_set=0.0, y_off_set=0.0, z_off_set=0.05, num_waypoints=1, roll=0.0, pitch=1.57, yaw=0.0, duration=2),
										transitions={'reached': 'move_x_dxn_only2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:788 y:358
			OperatableStateMachine.add('move_x_dxn_only2',
										MoveIn_auto_StraightPath_1dxn(x_off_set=-0.3, y_off_set=0.0, z_off_set=0.0, num_waypoints=3, roll=0.0, pitch=1.57, yaw=0.0, duration=3),
										transitions={'reached': 'move_to_default', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:346 y:162
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'move_to_detect1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'detected_x': 'detected_x', 'detected_y': 'detected_y', 'detected_z': 'detected_z'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
