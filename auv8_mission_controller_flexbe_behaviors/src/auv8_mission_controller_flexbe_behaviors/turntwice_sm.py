#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.RotateYawRelative import RotateYawRelative
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 07 2020
@author: fcr
'''
class turnTwiceSM(Behavior):
	'''
	720
	'''


	def __init__(self):
		super(turnTwiceSM, self).__init__()
		self.name = 'turnTwice'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:436 y:292
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:208 y:98
			OperatableStateMachine.add('turnYaw',
										RotateYawRelative(relativeYaw=179),
										transitions={'continue': 'yaw2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:491 y:101
			OperatableStateMachine.add('yaw2',
										RotateYawRelative(relativeYaw=179),
										transitions={'continue': 'yaw3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:674 y:331
			OperatableStateMachine.add('yaw3',
										RotateYawRelative(relativeYaw=179),
										transitions={'continue': 'yaw4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:376 y:507
			OperatableStateMachine.add('yaw4',
										RotateYawRelative(relativeYaw=179),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
