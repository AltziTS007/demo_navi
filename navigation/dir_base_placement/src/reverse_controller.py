#!/usr/bin/env python
import rospy
import gazebo_msgs.msg
import time
import tf2_ros
import pdb
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import geometry_msgs.msg
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0


def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rospy.loginfo(" x --->  %lf", x)
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
rospy.Subscriber("/odom", Odometry, newOdom)
rospy.init_node("reverse_controller")

pub = rospy.Publisher("/gaz_controller/cmd_vel", Twist, queue_size = 1)


speed = Twist()

r = rospy.Rate(4)



goal = Point()
goal.x = 0.008
goal.y = 5.763

while not rospy.is_shutdown():
	
	inc_x = goal.x -x
	inc_y = goal.y -y

	angle_to_goal = atan2(inc_y, inc_x)
	g_x = goal.x - x
	rospy.loginfo("goal.x - x --->  %lf", x)

	if abs(goal.x - x) >= 0.08:
		if abs(goal.x - x) > 0.08:
			speed.linear.x = -0.15
			speed.linear.y = 0.0
			speed.angular.z = 0.0
		elif abs(angle_to_goal - theta) > 0.1:
			speed.linear.x = 0.0
			speed.linear.y = 0.0
			speed.angular.z = 0.35
		elif abs(goal.y - y) > 0.05:
			speed.linear.x = 0.0
			speed.linear.y = 0.15
			speed.angular.z = 0.0
		pub.publish(speed)

	else:
		speed.linear.x = 0.0
		speed.linear.y = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		rospy.loginfo("MISION SUCK-SHEESH :)")
		break
	
	r.sleep() 
