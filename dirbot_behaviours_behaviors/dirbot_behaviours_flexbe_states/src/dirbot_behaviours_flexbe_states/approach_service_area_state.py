#!/usr/bin/env python

import rospy, yaml
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from rospy_message_converter import message_converter
from tf.transformations import euler_from_quaternion
from math import atan2

'''
Created: 22/06/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class ApproachServiceAreaState(EventState):
	'''
	State to better approach a service area in the simulation environment. Uses a virtual object's pose to 
	calculate the robot's distance and orientation relative to that object and publish cmd_vel messages in 
	order to correct the robot pose.

	># workstation_name  str  string containing the name of the current workstation goal.
	
	<= success
	<= failed
	'''

	def __init__(self):
		super(ApproachServiceAreaState, self).__init__(input_keys=['workstation_name'], outcomes=['success','failed'])
		
		self.semantic_map_path = '/home/smilon/smilon_ws/src/semantic_map/semantic_map.yaml'
		
		self._sub_topic = '/odom/'
		self._pub_topic = '/gaz_controller/cmd_vel'
		self._odom_sub = ProxySubscriberCached({self._sub_topic: Odometry})
		self._cmd_vel_pub = ProxyPublisher({self._pub_topic: Twist})

		self.curr_theta = None
		self.rate = rospy.Rate(2)


	def on_start(self):
		with open(self.semantic_map_path, 'r') as stream:
		    try:
			semantic_map = yaml.safe_load(stream)
		    except yaml.YAMLError as exc:
			Logger.logerror(exc)

		self.semantic_map = {k: v for d in semantic_map for k, v in d.items()} # convert list of dictionaries to dictionary
		

	def on_enter(self, userdata):
		ws_name = userdata.workstation_name
		target_pose = self.semantic_map[ws_name]['box_pose']
		self.target_pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', target_pose)
		

	def execute(self, userdata):

		if self._odom_sub.has_msg(self._sub_topic):  # get current 2D pose
			msg = self._odom_sub.get_last_msg(self._sub_topic)
			self.curr_x = msg.pose.pose.position.x
			self.curr_y = msg.pose.pose.position.y

			rot_q = msg.pose.pose.orientation
			_, _, self.curr_theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
			Logger.loginfo('current theta: {}'.format(self.curr_theta))
		else:   
			Logger.logerr('/odom topic message not received. Restart Gazebo.')
			return 'failed'


		if self.curr_theta is not None:
			speed = Twist()
			while not rospy.is_shutdown():
				x_diff = self.target_pose.position.x - self.curr_x
				Logger.loginfo('x_diff: {}'.format(x_diff))
				y_diff = self.target_pose.position.y - self.curr_y

				angle_to_goal = atan2(y_diff, x_diff)
				Logger.loginfo('Angle to goal: {}'.format(angle_to_goal))

				if abs(angle_to_goal - self.curr_theta) > 0.1:
					speed.linear.x = 0.0
					speed.linear.y = 0.0
					speed.angular.z = 0.35
				elif abs(x_diff) > 0.37:
					speed.linear.x = 0.2
					speed.linear.y = 0.0
					speed.angular.z = 0.0
				elif abs(y_diff) > 0.05:
					speed.linear.x = 0.0
					speed.linear.y = 0.2
					speed.angular.z = 0.0

				self._cmd_vel_pub.publish(self._pub_topic, speed)

				if abs(x_diff) <= 0.37:
					speed.linear.x = 0.0
					speed.linear.y = 0.0
					speed.angular.z = 0.0
					self._cmd_vel_pub.publish(self._pub_topic, speed)
					break
				
				self.rate.sleep()
			return 'success'

		else:
			return 'failed'	
		
		
