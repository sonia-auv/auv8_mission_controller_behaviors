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
from auv8_mission_controller_flexbe_states.ToTest.switchControlMode import switchControlMode
from auv8_mission_controller_flexbe_states.ToTest.moveSpeed import MoveSpeed
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 02 2020
@author: fcr
'''
class infinit_speedSM(Behavior):
	'''
	speed w time but time not implemeted /bug
	'''


	def __init__(self):
		super(infinit_speedSM, self).__init__()
		self.name = 'infinit_speed'

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
										MoveZByStep(depth=1, step=0.5),
										transitions={'continue': 'switch_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:317 y:84
			OperatableStateMachine.add('switch_2',
										switchControlMode(mode=2),
										transitions={'continue': 'speed_time', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:573 y:178
			OperatableStateMachine.add('speed_time',
										MoveSpeed(speed=1, yaw=45),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
