#!/usr/bin/env python

import rospy, yaml
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from rospy_message_converter import message_converter
from atwork_commander_msgs.msg import Task, Object


'''
Created: 18/06/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class RefBoxParserState(EventState):
	''' 
	Robocup@Work referee box state. Parses useful info from generated task messages, maps generated object IDs
	to their respective object names and utilises semantic information to map workstation names to a 2D pose.
	Outputs useful information to a python dictionary to be used by the planner.  
	
	#> task  dict  python dictionary containing the parsed task with mapped workstations
		       to coordinates and object IDs to poses.

	<= BMT
	<= BTT
	<= error_parsing
	'''	

	def __init__(self):
		super(RefBoxParserState, self).__init__(outcomes=['BMT','BTT','error_parsing'], output_keys=['task'])

		self._topic = 'atwork_commander/task' 
		self.semantic_map_path = '/semantic_map/semantic_map.yaml'

		self.task_dict = None
		
		object_msg = Object()
		self.object_dict = {
				object_msg.F20_20_B:      'F20_20_B',
				object_msg.F20_20_G:      'F20_20_G',
				object_msg.AXIS:          'Axis',
				object_msg.BEARING_BOX:   'Bearing_Box',
				object_msg.S40_40_G:      'S40_40_G',
				object_msg.S40_40_B:      'S40_40_B',
				object_msg.M20:           'M20',
				object_msg.M30:           'M30',
				object_msg.M20_100:       'M20_100',
				object_msg.R20:           'R20',
				object_msg.DISTANCE_TUBE: 'DISTANCE_TUBE'
				}
				

	def on_start(self):
		
		with open(self.semantic_map_path, 'r') as stream:
		    try:
			semantic_map = yaml.safe_load(stream)
		    except yaml.YAMLError as exc:
			Logger.logerror(exc)
		
		self.semantic_map = {k: v for d in semantic_map for k, v in d.items()} # convert list of dictionaries to dictionary
		Logger.loginfo('Semantic map: {}'.format(self.semantic_map))
		Logger.loginfo('Object mappings: {}'.format(self.object_dict))
	

	def on_enter(self, userdata):
		_sub = ProxySubscriberCached({self._topic: Task})	

		while True:
			if _sub.has_msg(self._topic):
				task_msg = _sub.get_last_msg(self._topic)
				Logger.loginfo('Received task.')
				break
			else:
				Logger.logwarn('Refbox atwork_commander/task topic is not being published.')
				rospy.sleep(0.5)	
		
		task = message_converter.convert_ros_message_to_dictionary(task_msg)
		#Logger.loginfo('Task message: {}'.format(task))
		

		task_dict = {
				'arena_start_state': [],
				'arena_target_state': [],
				'type': [] } 
				
		for i, (k, v) in enumerate(task.items()):
			#Logger.loginfo('[{}] {}:{}'.format(i, k, v))
			
			if k == 'arena_start_state' or k == 'arena_target_state':
				for j in range(len(task[k])):
					objects = []
					#Logger.loginfo('{}'.format(task[k][j]))
					for n in range(len(task[k][j]['objects'])): 
						objects.append(task[k][j]['objects'][n]['object'])
					
					ws_name = str(task[k][j]['workstation_name'])
					ws_pose = self.semantic_map[ws_name]['ws_pose']
					task_dict[k].append({'workstation_name': ws_name, 'pose': ws_pose, 'objects': objects})
			
			elif k == 'type':
				task_dict[k] = str(task[k])
				self.task_type = task[k]	
		
		Logger.loginfo('Parsed dictionary: {}'.format(task_dict))
		self.task_dict = task_dict


	def execute(self, userdata):
		
		if self.task_dict is None:
			return 'error_parsing'
		else:
			userdata.task = self.task_dict

		return self.task_type
