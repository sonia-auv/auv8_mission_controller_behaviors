#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.moveGlobalZ import moveGlobalZ
from auv8_mission_controller_flexbe_states.moveGlobalYaw import moveGlobalYaw
from auv8_mission_controller_flexbe_states.moveGlobalXY import moveGlobalXY
from auv8_mission_controller_flexbe_states.moveRelativeYDecoupled import MoveRelativeYDecoupled
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 02 2020
@author: fcr
'''
class mixtestSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(mixtestSM, self).__init__()
		self.name = 'mix test'

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
			# x:161 y:99
			OperatableStateMachine.add('depth',
										moveGlobalZ(depth=1),
										transitions={'continue': 'yaw', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:506 y:194
			OperatableStateMachine.add('yaw',
										moveGlobalYaw(yaw=155),
										transitions={'continue': 'deadrecon', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:753 y:301
			OperatableStateMachine.add('deadrecon',
										moveGlobalXY(positionX=1, positionY=1),
										transitions={'continue': 'asdadsa', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:637 y:472
			OperatableStateMachine.add('asdadsa',
										MoveRelativeYDecoupled(distanceY=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
