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
from auv8_mission_controller_flexbe_states.ToTest.hydro import Hydro
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Feb 22 2020
@author: fcr
'''
class hydroSM(Behavior):
	'''
	dsadadsa
	'''


	def __init__(self):
		super(hydroSM, self).__init__()
		self.name = 'hydro'

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
			# x:151 y:53
			OperatableStateMachine.add('depth',
										MoveZByStep(depth=1, step=0.5),
										transitions={'continue': 'hydro', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:448 y:98
			OperatableStateMachine.add('hydro',
										Hydro(frequency=40),
										transitions={'continue': 'hydro_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:644 y:491
			OperatableStateMachine.add('hydro_2',
										Hydro(frequency=40),
										transitions={'continue': 'hydro', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
