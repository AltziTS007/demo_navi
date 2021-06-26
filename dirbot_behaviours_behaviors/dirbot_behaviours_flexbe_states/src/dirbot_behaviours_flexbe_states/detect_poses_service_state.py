#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import Pose

from dirbot_msgs.srv import DetectPoses, DetectPosesRequest


'''
Created: 07/05/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class DetectPoseState(EventState):
	'''
	A state for reporting detected objects, their location and 2D orientation utilising the 'detect_poses' service server.

	#> detection_names 		str 	Detected object names.
	#> detected_poses	        Pose    Detected objects pose, i.e 3D coordinates (w.r.t camera) and 2D orientation (YAW).
	#> object_idx                   int

	<= detections 		
	<= no_detections
	<= error	
	
	'''

	def __init__(self):
		# See example_state.py for basic explanations.
		super(DetectPoseState, self).__init__(outcomes = ['detections', 'no_detections', 'error'], output_keys=['detection_names','detected_poses','object_idx'])
												 
		self._topic = '/detect_pose'  # service topic
		self._client = ProxyServiceCaller({self._topic: DetectPoses}) # pass required clients as dict (topic: MsgType)

		self._error = False
		self._result = None


	def on_enter(self, userdata):
		# TODO: perhaps initialize service server from here.
		# Make service call.
		Logger.loginfo("ProxyServiceCaller")
		try:
			self._result = self._client.call(self._topic, DetectPosesRequest())
			Logger.loginfo("ProxyServiceCaller call status: {}".format(self._result))
		except Exception as e:
			Logger.logwarn('[{0}] Failed to call service: {1}'.format(self.name, str(e)))		
			
		
	def execute(self, userdata):
		# While this state is active, check if the service has been finished (and evaluate the result).
		# TODO: what happens if "no_detections"?
		
		if self._result is not None:
			# Check if service call has been finished
			userdata.detection_names = self._result.name
			userdata.detected_poses = self._result.pose
			userdata.object_idx = 0
			return 'detections'
		
		elif self._error:
			# Check if the client failed to make call
			return 'error'

	
