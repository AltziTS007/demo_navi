#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import Point

from dirbot_msgs.srv import DetectObjectsMsg, DetectObjectsMsgRequest


class DetectObjectsState(EventState):
	'''
	A state for reporting detected objects utilising the 'detect_objects' service server.

	#> detection_names 		str 	Detected object names.
	#> detection_points             Point   Detected object coordinates w.r.t camera location.

	<= detections 		
	<= no_detections
	<= error		
	'''

	def __init__(self):
		super(DetectObjectsState, self).__init__(outcomes = ['detections', 'no_detections', 'error'], output_keys=['detection_names','detection_points'])
												 
		self._topic = '/detect_objects'
		self._client = ProxyServiceCaller({self._topic: DetectObjectsMsg}) # pass required clients as dict (topic: type)

		self._error = False
		self._result = None


	def on_enter(self, userdata):
		# TODO: perhaps initialize service server from here.
		# Make service call.
		Logger.loginfo("ProxyServiceCaller")
		try:
			self._result = self._client.call(self._topic, DetectObjectsMsgRequest())
			Logger.loginfo("ProxyServiceCaller call status: {}".format(self._result))
		except Exception as e:
			Logger.logwarn('[{0}] Failed to call service: {1}'.format(self.name, str(e)))		
			
		
	def execute(self, userdata):
		# While this state is active, check if the service has been finished (and evaluate the result).
		# TODO: what happens if "no_detections"?

		# Check if the client failed to make call
		if self._error:
			return 'error'

		# Check if service call has been finished
		if self._result is not None:
			userdata.detection_names = self._result.name
			userdata.detection_points = self._result.position
			return 'detections'
		

	
