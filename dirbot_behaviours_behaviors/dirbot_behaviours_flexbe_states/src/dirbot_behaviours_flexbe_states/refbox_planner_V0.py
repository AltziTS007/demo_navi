#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose2D

'''
Created: 18/06/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class RefBoxPlannerStateV0(EventState):
	'''
	Robocup@Work referee box state. Takes a parsed task dictionary as input and iteratively extracts 
	an arena_start_state goal and then an arena_end_state goal by matching object lists to find target goal. 
	For each goal it communicates the workstation location information as a Pose-goal
	to the navigation states and the provided object lists to the object detection 
	state to prepare for a manipulation goal.

	Built to satisfy the Basic Manipulation Task (BMT) needs.

	># task              dict    python dictionary containing the parsed task with mapped workstations
		                     to coordinates and object IDs to poses.

	#> object_goal       list    contains current goal objects to be ideally detected by the object_detection state 
				     and eventually manipulated. 
	#> waywpoint         Pose2D  string containing the pose of the current workstation goal. To be used
				     for navigation purposes.
	#> workstation_name  str     string containing the name of the current workstation goal. To be used
				     for manipulation purposes.
	#> task              dict    updated task dictionary with current processed goal removed.

	<= task_iterated
	<= task_exhausted
	<= incorrect_task
	<= failed
	'''

	def __init__(self):
		super(RefBoxPlannerStateV0, self).__init__(input_keys=['task'], output_keys=['object_goal','waypoint','workstation_name','task'], 
							outcomes=['task_iterated','task_exhausted','incorrect_task','failed'])
	
		self.start = None     # if True, indicates that robot currently needs to process a pick goal


	def on_enter(self, userdata):
		self.task_dict = userdata.task # read updated task each time state is entered
		

	def execute(self, userdata):
		task_dict = self.task_dict
		
		_number_of_start_goals = len(task_dict['arena_start_state'])
		_number_of_target_goals = len(task_dict['arena_target_state'])
		
		if _number_of_start_goals == _number_of_target_goals and _number_of_start_goals != 0 and _number_of_target_goals != 0:
			self.start = True # Arena start_state condition
			workstation_name = task_dict['arena_start_state'][0]['workstation_name']
			workstation_goal = task_dict['arena_start_state'][0]['pose']
			object_goal = task_dict['arena_start_state'][0]['objects']
			Logger.loginfo('Arena_start_goal: {}'.format(workstation_name))
			
			userdata.workstation_name = workstation_name
			userdata.waypoint = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose2D', workstation_goal)
			userdata.object_goal = object_goal
			#Logger.loginfo('Workstation name: {} \n Workstation Goal:\n {} \n Object Goal: {} '.format(userdata.workstation_name, userdata.workstation_goal, userdata.object_goal))
			return 'task_iterated'

		elif _number_of_start_goals < _number_of_target_goals:
			self.start = False # Arena target_state condition
			
			remaining_objects_ss = []
			for key in task_dict['arena_start_state']:
				remaining_objects_ss.append(key['objects'])
			#Logger.loginfo('Remaining arena_start_state objects: {}'.format(remaining_objects_ss)) 

			remaining_objects_ts = []
			for key in task_dict['arena_target_state']:
				remaining_objects_ts.append(key['objects'])
			#Logger.loginfo('Remaining arena_target_state objects: {}'.format(remaining_objects_ts)) 	 
			
			for idx in range(len(remaining_objects_ts)): #  find missing values from start list (hinting that target goal should be missing object list)
				try:
					remaining_objects_ss.index(remaining_objects_ts[idx])
				except ValueError:
					self.idx = idx

			#Logger.loginfo('Next target goal: {}'.format(remaining_objects_ts[idx]))

			workstation_name = task_dict['arena_target_state'][idx]['workstation_name']
			workstation_goal = task_dict['arena_target_state'][idx]['pose']
			object_goal = task_dict['arena_target_state'][idx]['objects']
			Logger.loginfo('Arena_target_goal: {}'.format(workstation_name))
			
			userdata.workstation_name = workstation_name
			userdata.waypoint = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose2D', workstation_goal)
			userdata.object_goal = object_goal
			#Logger.loginfo('Workstation name: {} \n Workstation Goal:\n {} \n Object Goal: {} '.format(userdata.workstation_name, userdata.workstation_goal, userdata.object_goal))
			return 'task_iterated'

		elif _number_of_target_goals == 0:
			self.start = None 
			return 'task_exhausted'

		elif _number_of_start_goals > number_of_target_goals:
			return 'incorrect_task'

		elif self.start is None:
			return 'incorrect_task' 


	def on_exit(self, userdata):

		if self.start:
			del self.task_dict['arena_start_state'][0]
		elif not self.start and self.start is not None:
			del self.task_dict['arena_target_state'][self.idx]
		
		#Logger.loginfo('Updated task: {}'.format(self.task_dict))
		userdata.task = self.task_dict




