#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal

# Source: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html


## TODO:
# - Think about what happens on_exit/on_stop for failsafe reasons.
# - Current issue: "There is no end effector to set the pose for" - Could it be init_node issue

'''
Created: 25/05/2021

@author: Milton Logothetis
email: milton.logothetis@gmail.com
'''
class PickState(EventState):
	'''
	Pick state utilising MoveIt's move_group python interface to pick an object. The state waits 
	for a geometry_msgs/Pose() goal and plans a RANDOM path to achieve that goal. If the pick maneuver fails 
	by allowing the gripper to fully close, indicating no object in the gripper, then the state fails otherwise
	it succeeds.

	># pose_goal  Pose  Pose goal array received from a computer vision approach.

	<= grasped  Joint location has reached pose_goal and gripper encoder has detected an object.  
	<= failed  Joint location not reached OR gripper encoder did not detect object.

	'''
	
	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(PickState, self).__init__(input_keys=['pose_goal'], outcomes=['grasped', 'failed'])

	def on_start(self):
		# Initialize
		moveit_commander.roscpp_initialize(sys.argv)
		#rospy.init_node('move_group_arm', anonymous=True)

	def on_enter(self, userdata):

                self._client = ProxyActionClient({'execute_trajectory': ExecuteTrajectoryAction})
		
		while not self._client.is_available('execute_trajectory'):
			rospy.sleep(1.)
		Logger.loginfo('client is available')

		self.eef_group = moveit_commander.MoveGroupCommander('widowx') #  specify end-effector group
		#self.eef_group.set_pose_target(userdata.pose_goal)	#####
		
		## Debug 
		planning_frame = self.eef_group.get_planning_frame()
		Logger.loginfo("============ Planning frame: {}".format(planning_frame))

		eef_link = self.eef_group.get_end_effector_link()
		Logger.loginfo("============ End effector link: {}".format(eef_link))
		
		#self.eef_group.get_current_pose(eef_link) # Get the current pose of the end-effector of the group. Throws an exception if there is not end-effector. 

						
	def execute(self, userdata):
		## Plan (TODO: Put plan in on_enter??)
		
		self.eef_group.set_named_target("pick")	
		eef_goal = ExecuteTrajectoryGoal()
		plan = self.eef_group.plan()
		eef_goal.trajectory = plan
		self._client.send_goal('execute_trajectory', eef_goal)
		
		rospy.sleep(15.) # trying to simulate client.wait_for_result()
		return 'grasped'
		#plan = self.eef_group.go(wait=True)
		#self.eef_group.stop()
		#self.eef_group.clear_pose_targets()

		## Execute
		#self.eef_group.execute(plan, wait=True)
		
		
				
		## TODO: Get actual end-effector location and compare with Goal. 
		# 1. IF goal -> reached close gripper	
		# 2. Access gripper encoder. IF encoder value constant and gripper is not fully closed for e.g. <0.5s THEN object has been grasped -> RETURN succeeded. ELSE RETURN failed and go back to 			pose_detection.
			
	#def on_exit(self):
		#self.eef_group.stop()
		#self.eef_group.clear_pose_targets()
	
	#def on_stop(self):
		#self.eef_group.stop()
		#self.eef_group.clear_pose_targets()

