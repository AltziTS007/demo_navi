#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dirbot_behaviours_flexbe_states.detect_poses_service_state import DetectPoseState
from dirbot_behaviours_flexbe_states.iterator_state import PoseIteratorState
from dirbot_behaviours_flexbe_states.pick_state import PickState
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 06 2021
@author: Milton Logothetis
'''
class DetectPosesDevSM(Behavior):
	'''
	Detects objects in the camera frame using the YOLO network and computes their 3D location in space using depth data. Additionally, outputs 2D orientation (YAW) of objects and has optimal performance when the camera is pointed at the objects in a top-view.
	'''


	def __init__(self):
		super(DetectPosesDevSM, self).__init__()
		self.name = 'DetectPosesDev'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1206 y:150, x:679 y:307
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:129 y:157
			OperatableStateMachine.add('DetectPoseState',
										DetectPoseState(),
										transitions={'detections': 'LogNames', 'no_detections': 'LogNoDetections', 'error': 'LogError'},
										autonomy={'detections': Autonomy.Off, 'no_detections': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'detection_names': 'detection_names', 'detected_poses': 'detected_poses', 'object_idx': 'object_idx'})

			# x:398 y:294
			OperatableStateMachine.add('LogError',
										LogState(text='An error was encountered in the DetectPoseState process.', severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:396 y:64
			OperatableStateMachine.add('LogNames',
										LogKeyState(text='Predicted object names: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'LogPoses'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'detection_names'})

			# x:166 y:290
			OperatableStateMachine.add('LogNoDetections',
										LogState(text='No objects were detected in the scene.', severity=Logger.REPORT_HINT),
										transitions={'done': 'DetectPoseState'},
										autonomy={'done': Autonomy.Off})

			# x:544 y:64
			OperatableStateMachine.add('LogPoses',
										LogKeyState(text='Predicted pose: {}', severity=Logger.REPORT_HINT),
										transitions={'done': 'PoseIterator'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'detected_poses'})

			# x:714 y:63
			OperatableStateMachine.add('PoseIterator',
										PoseIteratorState(),
										transitions={'iterated': 'ComputePick', 'exhausted': 'finished'},
										autonomy={'iterated': Autonomy.Off, 'exhausted': Autonomy.Off},
										remapping={'detection_names': 'detection_names', 'detected_poses': 'detected_poses', 'object_idx': 'object_idx', 'pose_goal': 'pose_goal'})

			# x:870 y:174
			OperatableStateMachine.add('ComputePick',
										PickState(),
										transitions={'grasped': 'PoseIterator', 'failed': 'DetectPoseState'},
										autonomy={'grasped': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose_goal': 'pose_goal'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
