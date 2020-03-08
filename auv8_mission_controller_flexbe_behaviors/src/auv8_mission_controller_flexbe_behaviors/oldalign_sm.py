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
from auv8_mission_controller_flexbe_states.alignToVision import AlignToVision
from auv8_mission_controller_flexbe_states.forwardVision import ForwardVision
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 07 2020
@author: fcr
'''
class oldAlignSM(Behavior):
	'''
	alignement
	'''


	def __init__(self):
		super(oldAlignSM, self).__init__()
		self.name = 'oldAlign'

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
			# x:81 y:39
			OperatableStateMachine.add('depth',
										MoveZByStep(depth=1.5, step=0.5),
										transitions={'continue': 'align poche', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:433 y:103
			OperatableStateMachine.add('align poche',
										AlignToVision(bonding_box_in_pixel=200, target_width_in_meter=0.30, topic_to_listen='/proc_image_processing/simple_vampire_torpille_result', pixel_to_victory=300, nb_max_align=10, max_queue_size=10, control_bonding_box_y=200),
										transitions={'continue': 'finished', 'failed': 'failed', 'forward': 'forward'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'forward': Autonomy.Off})

			# x:785 y:167
			OperatableStateMachine.add('forward',
										ForwardVision(distance_forward=1, initial_bondingBox=600, final_bondingBox=100, threshold_width=100, target_width=0.3, pixel_victory=400, topic_to_listen='/proc_image_processing/simple_vampire_torpille_result'),
										transitions={'continue': 'finished', 'failed': 'align poche', 'lost': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'lost': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
