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
transform = geometry_msgs.msg.TransformStamped()


def callback(data):  
	global transform
	transform.child_frame_id = "unit_box"
	for i in range(len(data.name)):
		if (transform.child_frame_id == data.name[i]):

			transform.transform.translation.x = data.pose[i].position.x
			transform.transform.translation.y = data.pose[i].position.y
			transform.transform.rotation.w = data.pose[i].orientation.w
			transform.transform.rotation.z = data.pose[i].orientation.z
    
	print(transform)
rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)

def newOdom(msg):
	global x
	global y
	global theta

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("base_placement")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/gaz_controller/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)



goal = Point()
goal.x = transform.transform.translation.x
goal.y = transform.transform.translation.y

while not rospy.is_shutdown():
	inc_x = goal.x -x
	inc_y = goal.y -y

	angle_to_goal = atan2(inc_y, inc_x)

	if abs(angle_to_goal - theta) > 0.1:
		speed.linear.x = 0.0
		speed.linear.y = 0.0
		speed.angular.z = 0.35
	elif abs(goal.x - x) > 0.37:
		speed.linear.x = 0.2
		speed.linear.y = 0.0
		speed.angular.z = 0.0
	elif abs(goal.y - y) > 0.05:
		speed.linear.x = 0.0
		speed.linear.y = 0.2
		speed.angular.z = 0.0

	pub.publish(speed)

	if (abs(goal.x - x) <= 0.37):
		speed.linear.x = 0.0
		speed.linear.y = 0.0
		speed.angular.z = 0.0
		pub.publish(speed)
		rospy.loginfo("MISION SUCK-SHEESH :)")
		break
	
	r.sleep()

