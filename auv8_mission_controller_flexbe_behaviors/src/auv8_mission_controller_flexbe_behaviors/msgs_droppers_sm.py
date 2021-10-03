#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.dropperState import Droppers
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 10 2021
@author: .
'''
class msgs_droppersSM(Behavior):
	'''
	.
	'''


	def __init__(self):
		super(msgs_droppersSM, self).__init__()
		self.name = 'msgs_droppers'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:689 y:324, x:717 y:266
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:131 y:98
			OperatableStateMachine.add('msg_droppers',
										Droppers(param=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
