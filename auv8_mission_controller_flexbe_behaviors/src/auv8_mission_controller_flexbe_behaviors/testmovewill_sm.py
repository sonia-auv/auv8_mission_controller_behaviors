#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.UnaprouvedState.MoveComplete import MoveComplete
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 30 2021
@author: William Brouillard
'''
class TestMoveWillSM(Behavior):
	'''
	Test for a movement state
	'''


	def __init__(self):
		super(TestMoveWillSM, self).__init__()
		self.name = 'TestMoveWill'

		# parameters of this behavior
		self.add_parameter('MoveX', 0)
		self.add_parameter('MoveY', 0)
		self.add_parameter('MoveZ', 0)
		self.add_parameter('RotX', 0)
		self.add_parameter('RotY', 0)
		self.add_parameter('RotZ', 0)
		self.add_parameter('Frame', 1)
		self.add_parameter('Time', 10)
		self.add_parameter('Fine', 0)
		self.add_parameter('Rotation', True)

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
			# x:104 y:97
			OperatableStateMachine.add('move',
										MoveComplete(MoveX=self.MoveX, MoveY=self.MoveY, MoveZ=self.MoveZ, RotX=self.RotX, RotY=self.RotY, RotZ=self.RotZ, Frame=self.Frame, Time=self.Time, Fine=self.Fine, Rotation=self.Rotation),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
