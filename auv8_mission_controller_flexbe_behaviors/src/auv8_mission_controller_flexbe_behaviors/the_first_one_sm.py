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
from auv8_mission_controller_flexbe_states.RotateYawRelative import RotateYawRelative
from auv8_mission_controller_flexbe_states.moveXRelatif import MoveXRelative
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Feb 22 2020
@author: fcr
'''
class TheFirstOneSM(Behavior):
	'''
	This behavior make a square of size 2m, Rotating right
	'''


	def __init__(self):
		super(TheFirstOneSM, self).__init__()
		self.name = 'The First One'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# ! 40 30 
		# This is a comment



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:78 y:73
			OperatableStateMachine.add('Depth',
										MoveZByStep(depth=1.5, step=0.5),
										transitions={'continue': 'move x 2m', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:658 y:61
			OperatableStateMachine.add('Rotate right',
										RotateYawRelative(relativeYaw=90),
										transitions={'continue': 'move 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:656 y:152
			OperatableStateMachine.add('move 2',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'rotate 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:655 y:303
			OperatableStateMachine.add('move 3',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'rotate 3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:359 y:67
			OperatableStateMachine.add('move x 2m',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'Rotate right', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:655 y:214
			OperatableStateMachine.add('rotate 2',
										RotateYawRelative(relativeYaw=90),
										transitions={'continue': 'move 3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:658 y:392
			OperatableStateMachine.add('rotate 3',
										RotateYawRelative(relativeYaw=90),
										transitions={'continue': 'move 4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:374 y:401
			OperatableStateMachine.add('move 4',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
