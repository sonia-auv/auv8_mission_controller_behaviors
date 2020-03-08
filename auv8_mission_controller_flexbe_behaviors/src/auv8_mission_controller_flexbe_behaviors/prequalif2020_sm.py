#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from auv8_mission_controller_flexbe_states.timeOut import TimesOut
from auv8_mission_controller_flexbe_states.moveXRelatif import MoveXRelative
from auv8_mission_controller_flexbe_states.moveSpeedRelativeXWithSwitch import MoveRelativeSpeedXWithSwitch
from auv8_mission_controller_flexbe_states.moveGlobalXY import MoveXY
from auv8_mission_controller_flexbe_states.moveGlobalYaw import MoveYaw
from auv8_mission_controller_flexbe_states.moveGlobalZ import MoveZ
from auv8_mission_controller_flexbe_states.moveZByStep import MoveZByStep
from auv8_mission_controller_flexbe_states.switchControlMode import Switch
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 07 2020
@author: fcr
'''
class prequalif2020SM(Behavior):
	'''
	speed prequalif pour babe
	'''


	def __init__(self):
		super(prequalif2020SM, self).__init__()
		self.name = 'prequalif2020'

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
			# x:83 y:54
			OperatableStateMachine.add('timer poche',
										TimesOut(duration=15),
										transitions={'continue': 'initialSwitch', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:850 y:67
			OperatableStateMachine.add('pass gate',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'speed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1006 y:191
			OperatableStateMachine.add('speed',
										MoveRelativeSpeedXWithSwitch(distance=10, speed=1, yaw=350),
										transitions={'continue': 'speedTurn', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:852 y:304
			OperatableStateMachine.add('speedTurn',
										MoveRelativeSpeedXWithSwitch(distance=4, speed=1, yaw=90),
										transitions={'continue': 'speed back', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:531 y:360
			OperatableStateMachine.add('speed back',
										MoveRelativeSpeedXWithSwitch(distance=10, speed=1, yaw=190),
										transitions={'continue': 'mode 0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:643 y:527
			OperatableStateMachine.add('moveglobal',
										MoveXY(positionX=2, positionY=0),
										transitions={'continue': 'yawGlobal', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:327 y:612
			OperatableStateMachine.add('yawGlobal',
										MoveYaw(yaw=180),
										transitions={'continue': 'pass gate back', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:62 y:552
			OperatableStateMachine.add('pass gate back',
										MoveXRelative(distance_x=2),
										transitions={'continue': 'scareFrancis', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:36 y:458
			OperatableStateMachine.add('scareFrancis',
										MoveZ(depth=0),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:439 y:76
			OperatableStateMachine.add('setDepth',
										MoveZByStep(depth=1, step=0.5),
										transitions={'continue': 'yawglob', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:777 y:435
			OperatableStateMachine.add('mode 0',
										Switch(mode=0),
										transitions={'continue': 'moveglobal', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:50 y:191
			OperatableStateMachine.add('initialSwitch',
										Switch(mode=0),
										transitions={'continue': 'setDepth', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:595 y:192
			OperatableStateMachine.add('yawglob',
										MoveYaw(yaw=310),
										transitions={'continue': 'pass gate', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
