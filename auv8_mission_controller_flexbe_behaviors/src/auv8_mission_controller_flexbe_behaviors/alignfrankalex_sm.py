#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.moveZByStep import MoveZByStep
from auv8_mission_controller_flexbe_states.alignAlexFrank import AlignAlexFrank
from auv8_mission_controller_flexbe_states.vision import LaunchVision
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 07 2020
@author: fcr
'''
class alignFrankAlexSM(Behavior):
	'''
	align front speed
	'''


	def __init__(self):
		super(alignFrankAlexSM, self).__init__()
		self.name = 'alignFrankAlex'

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
			# x:30 y:40
			OperatableStateMachine.add('depth',
										MoveZByStep(depth=1.5, step=0.5),
										transitions={'continue': 'launchFront', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:471 y:155
			OperatableStateMachine.add('the best align',
										AlignAlexFrank(object_height=450, object_width=300, distance_to_target=1, yaw_adjustment=10, max_time=100, topic_to_listen='/proc_image_processing/execution_953', image_height=1544, image_width=2064, speed=0.1, maximum_alignment=10, queue_size=10),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:323 y:43
			OperatableStateMachine.add('launchFront',
										LaunchVision(param_node_name='execution_953', camera_no=1, param_cmd=1),
										transitions={'continue': 'the best align', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
