#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.thrusterControlManual import ThrusterControlManual
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 07 2020
@author: Alex
'''
class TestControlManualSM(Behavior):
	'''
	Test thrusters manual control.
	'''


	def __init__(self):
		super(TestControlManualSM, self).__init__()
		self.name = 'TestControlManual'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:658 y:244, x:649 y:367
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:236 y:285
			OperatableStateMachine.add('ManualControl',
										ThrusterControlManual(bag_name='Allo.bag', bag_time=10, T1=10, T2=0, T3=0, T4=0, T5=0, T6=0, T7=0, T8=0),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
