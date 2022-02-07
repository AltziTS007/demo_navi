#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dirbot_behaviours_flexbe_states.refbox_parser_state import RefBoxParserState
from dirbot_behaviours_flexbe_states.refbox_planner_state import RefBoxPlannerState
from flexbe_navigation_states.move_base_state import MoveBaseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 15 2021
@author: Milton Logothetis
'''
class refbox_testSM(Behavior):
	'''
	refbox tester behaviour
	'''


	def __init__(self):
		super(refbox_testSM, self).__init__()
		self.name = 'refbox_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:88 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:198 y:115
			OperatableStateMachine.add('refbox_parser',
										RefBoxParserState(),
										transitions={'BMT': 'refbox_planner', 'BTT': 'refbox_planner', 'error_parsing': 'failed'},
										autonomy={'BMT': Autonomy.Off, 'BTT': Autonomy.Off, 'error_parsing': Autonomy.Off},
										remapping={'task': 'task'})

			# x:293 y:257
			OperatableStateMachine.add('refbox_planner',
										RefBoxPlannerState(),
										transitions={'task_iterated': 'move_base', 'task_exhausted': 'finished', 'incorrect_task': 'refbox_parser', 'failed': 'failed'},
										autonomy={'task_iterated': Autonomy.Low, 'task_exhausted': Autonomy.Off, 'incorrect_task': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'task': 'task', 'object_goal': 'object_goal', 'waypoint': 'waypoint', 'workstation_name': 'workstation_name'})

			# x:565 y:171
			OperatableStateMachine.add('move_base',
										MoveBaseState(),
										transitions={'arrived': 'refbox_planner', 'failed': 'refbox_planner'},
										autonomy={'arrived': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
