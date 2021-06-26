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
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from flexbe_states.log_key_state import LogKeyState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 06 2021
@author: Milton Logothetis
'''
class DetectPosesDevCustomSM(Behavior):
	'''
	Detects objects in the camera frame using the YOLO network and computes their 3D location in space using depth data. Additionally, outputs 2D orientation (YAW) of objects and has optimal performance when the camera is pointed at the objects in a top-view.
	'''


	def __init__(self):
		super(DetectPosesDevCustomSM, self).__init__()
		self.name = 'DetectPosesDevCustom'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1220 y:69, x:675 y:339
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:463, x:130 y:463
		_sm_custompickplace_0 = OperatableStateMachine(outcomes=['reached', 'param_error'])

		with _sm_custompickplace_0:
			# x:30 y:40
			OperatableStateMachine.add('HomeArm',
										SrdfStateToMoveit(config_name="watch", move_group="widowx", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GripperOpenPick', 'planning_failed': 'HomeArm', 'control_failed': 'HomeArm', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Full, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:81 y:315
			OperatableStateMachine.add('CustomPlace',
										SrdfStateToMoveit(config_name="place", move_group="widowx", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GripperOpenPlace', 'planning_failed': 'CustomPlace', 'control_failed': 'CustomPlace', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Full, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:321 y:236
			OperatableStateMachine.add('GripperClose',
										SrdfStateToMoveit(config_name="close", move_group="hand", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'CustomPlace', 'planning_failed': 'GripperClose', 'control_failed': 'GripperClose', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:304 y:58
			OperatableStateMachine.add('GripperOpenPick',
										SrdfStateToMoveit(config_name="open", move_group="hand", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'CustomPick', 'planning_failed': 'GripperOpenPick', 'control_failed': 'GripperOpenPick', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:267 y:388
			OperatableStateMachine.add('GripperOpenPlace',
										SrdfStateToMoveit(config_name="open", move_group="hand", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'reached', 'planning_failed': 'GripperOpenPlace', 'control_failed': 'GripperOpenPlace', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:108 y:162
			OperatableStateMachine.add('CustomPick',
										SrdfStateToMoveit(config_name="pick", move_group="widowx", action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GripperClose', 'planning_failed': 'CustomPick', 'control_failed': 'CustomPick', 'param_error': 'param_error'},
										autonomy={'reached': Autonomy.Full, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})



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

			# x:722 y:59
			OperatableStateMachine.add('PoseIterator',
										PoseIteratorState(),
										transitions={'iterated': 'CustomPickPlace', 'exhausted': 'finished'},
										autonomy={'iterated': Autonomy.Off, 'exhausted': Autonomy.Off},
										remapping={'detection_names': 'detection_names', 'detected_poses': 'detected_poses', 'object_idx': 'object_idx', 'pose_goal': 'pose_goal'})

			# x:895 y:258
			OperatableStateMachine.add('CustomPickPlace',
										_sm_custompickplace_0,
										transitions={'reached': 'PoseIterator', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Inherit, 'param_error': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
