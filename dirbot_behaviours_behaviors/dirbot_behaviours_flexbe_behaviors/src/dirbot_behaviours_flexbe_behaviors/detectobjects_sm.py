#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dirbot_behaviours_flexbe_states.detect_objects_service_state import DetectObjectsState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 12 2021
@author: Milton Logothetis
'''
class DetectObjectsSM(Behavior):
	'''
	Object detection behaviour. Involves calling object_detection_service_server.py and outputs 3D location points of detected objects alongside the predicted names.
	'''


	def __init__(self):
		super(DetectObjectsSM, self).__init__()
		self.name = 'DetectObjects'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:972 y:84, x:699 y:457
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:187 y:108
			OperatableStateMachine.add('DetectObjects',
										DetectObjectsState(),
										transitions={'detections': 'LogNames', 'no_detections': 'LogNoDetections', 'error': 'failed'},
										autonomy={'detections': Autonomy.Off, 'no_detections': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'detection_names': 'detection_names', 'detection_points': 'detection_points'})

			# x:584 y:36
			OperatableStateMachine.add('LogNames',
										LogKeyState(text='Prediction NAMES: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'LogPoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'detection_names'})

			# x:586 y:164
			OperatableStateMachine.add('LogNoDetections',
										LogState(text='No predictions in current scene.', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:737 y:36
			OperatableStateMachine.add('LogPoints',
										LogKeyState(text='Prediction POINTS: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'detection_points'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
