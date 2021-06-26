#!/usr/bin/env python
from flexbe_core import EventState, Logger
import rospy
from geometry_msgs.msg import Pose


'''
Created: 26/05/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class PoseIteratorState(EventState):
	'''
	Receives a Pose() message and an index and iterates it. Useful for iterating under certain message conditions.
	I.e if detection_names message field is empty do not pass it on to the manipulation states.
	
	>#  detection_names  string[]
	>#  detected_poses   Pose[]
	>#  object_idx       int     Index indicating the target object from the pose_goal.
	
	#>  object_idx       int
	#>  pose_goal        Pose
	
	<=  iterated
	<=  exhausted

	'''


	def __init__(self):
		super(PoseIteratorState, self).__init__(input_keys=['detection_names','detected_poses','object_idx'], output_keys=['object_idx','pose_goal'], outcomes=['iterated','exhausted'])\


	def on_enter(self, userdata):
		self._detection_names = userdata.detection_names
		self._detected_poses = userdata.detected_poses
		self._object_idx = userdata.object_idx

		Logger.loginfo('Input pose goal: \n{}'.format(userdata.detected_poses))


	def execute(self, userdata):
		try:
			if self._detection_names[self._object_idx] == ['']:
				return 'exhausted'
			else:  
				# Select pose goal
				userdata.pose_goal = Pose()
				userdata.pose_goal.position.x = self._detected_poses[self._object_idx].position.x
				userdata.pose_goal.position.y = self._detected_poses[self._object_idx].position.y
				userdata.pose_goal.position.z = self._detected_poses[self._object_idx].position.z
				# TODO: SHOULD ADD orientation.x.y.z ??
				userdata.pose_goal.orientation.z = self._detected_poses[self._object_idx].orientation.z
				userdata.pose_goal.orientation.w = self._detected_poses[self._object_idx].orientation.w
				
				# Iterate index
				userdata.object_idx += 1

				Logger.loginfo('Output pose goal (idx={}):\n {}'.format(userdata.object_idx, userdata.pose_goal))
				return 'iterated'
		except:
			return 'exhausted'
		
