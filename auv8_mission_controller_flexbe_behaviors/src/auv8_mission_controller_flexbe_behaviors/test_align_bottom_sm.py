#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.vision import vision
from auv8_mission_controller_flexbe_states.alignRoulette import AlignRoulette
from auv8_mission_controller_flexbe_states.forwardBottom import ForwardBottom
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Mar 05 2020
@author: fcr
'''
class testalignbottomSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(testalignbottomSM, self).__init__()
		self.name = 'test align bottom'

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
			# x:120 y:73
			OperatableStateMachine.add('bot',
										vision(param_node_name='simple_bat_wolf', camera_no=2, param_cmd=1),
										transitions={'continue': 'align', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:379 y:81
			OperatableStateMachine.add('align',
										AlignRoulette(topic_to_listen='/proc_image_processing/simple_vampire_torpille_result', bounding_box=100, target_width=0.15, target_height=1.2, max_queue_size=10, control_bounding_box=0.5, nb_pixel_to_victory=375),
										transitions={'continue': 'finished', 'failed': 'failed', 'forward': 'forward'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'forward': Autonomy.Off})

			# x:573 y:242
			OperatableStateMachine.add('forward',
										ForwardBottom(distance_z=0.5, bonding_box=200, depth_max=2.5, pixel_to_victory=200, topic_to_listen='/proc_image_processing/simple_vampire_torpille_result'),
										transitions={'continue': 'finished', 'failed': 'failed', 'aborted': 'align'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'aborted': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
