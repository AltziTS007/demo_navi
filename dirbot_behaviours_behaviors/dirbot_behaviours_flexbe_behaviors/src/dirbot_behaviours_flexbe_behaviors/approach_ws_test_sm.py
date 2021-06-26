#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dirbot_behaviours_flexbe_states.approach_service_area_state import ApproachServiceAreaState
from dirbot_behaviours_flexbe_states.refbox_parser_state import RefBoxParserState
from dirbot_behaviours_flexbe_states.refbox_planner_state import RefBoxPlannerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 22 2021
@author: Milton
'''
class approach_ws_testSM(Behavior):
	'''
	Approach workstation state test
	'''


	def __init__(self):
		super(approach_ws_testSM, self).__init__()
		self.name = 'approach_ws_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:170 y:348, x:284 y:350
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:77
			OperatableStateMachine.add('refbox_parser',
										RefBoxParserState(),
										transitions={'BMT': 'refbox_planner', 'BTT': 'refbox_planner', 'error_parsing': 'failed'},
										autonomy={'BMT': Autonomy.Off, 'BTT': Autonomy.Off, 'error_parsing': Autonomy.Off},
										remapping={'task': 'task'})

			# x:388 y:84
			OperatableStateMachine.add('refbox_planner',
										RefBoxPlannerState(),
										transitions={'task_iterated': 'ApproachServiceAreaState', 'task_exhausted': 'finished', 'incorrect_task': 'failed', 'failed': 'failed'},
										autonomy={'task_iterated': Autonomy.Off, 'task_exhausted': Autonomy.Off, 'incorrect_task': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'task': 'task', 'object_goal': 'object_goal', 'waypoint': 'waypoint', 'workstation_name': 'workstation_name'})

			# x:471 y:245
			OperatableStateMachine.add('ApproachServiceAreaState',
										ApproachServiceAreaState(),
										transitions={'success': 'failed', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'workstation_name': 'workstation_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
