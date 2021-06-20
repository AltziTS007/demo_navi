#!/usr/bin/env python
# license removed for brevity
import rospy
import tf2_ros
import gazebo_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import time
import pdb
import math

if __name__ == '__main__':
	rospy.init_node('alligner', anonymous=True)
    
	def quaternion_to_euler(x, y, z, w):

		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.degrees(math.atan2(t0, t1))

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.degrees(math.atan2(t3, t4))

		return X, Y, Z
	
	def callback(data):
		pub = rospy.Publisher('/gaz_controller/cmd_vel', Twist, queue_size=100)
		vel_msg = Twist()
  
		transform = geometry_msgs.msg.TransformStamped()
		transform.child_frame_id = "unit_box"
		for i in range(len(data.name)):
			if (transform.child_frame_id == data.name[i]):

				transform.transform.translation.x = data.pose[i].position.x
				transform.transform.translation.y = data.pose[i].position.y
				transform.transform.rotation.w = data.pose[i].orientation.w
				transform.transform.rotation.z = data.pose[i].orientation.z
    
		print(transform)
		print("---")
		transform1 = geometry_msgs.msg.TransformStamped()
		transform1.child_frame_id = "demo_DIR"
		for i in range(len(data.name)):
			if (transform1.child_frame_id == data.name[i]):

				transform1.transform.translation.x = data.pose[i].position.x
				transform1.transform.translation.y = data.pose[i].position.y
				transform1.transform.rotation.w = data.pose[i].orientation.w
				transform1.transform.rotation.z = data.pose[i].orientation.z

		print(transform1)
		print("---")

		r_O,p_O,y_O = quaternion_to_euler(0, 0, transform.transform.rotation.z, transform.transform.rotation.w)
		r_R,p_R,y_R = quaternion_to_euler(0, 0, transform1.transform.rotation.z, transform1.transform.rotation.w)

		print("robot", y_R)
		print("---")
		print("object", y_O)
		
		
		if (abs(y_O - y_R) > 0.00001):
			print("rotation", y_O - y_R)
			vel_msg.angular.z = 0.3
			pub.publish(vel_msg)
		if (abs(y_O - y_R) < 0.1):
			vel_msg.angular.z = 0
			pub.publish(vel_msg)
			rospy.sleep(3)

		if (abs(transform.transform.translation.y - transform1.transform.translation.y) > 0.001):
			print("linear.y")
			vel_msg.linear.y = 0.1
			pub.publish(vel_msg)
		elif (transform.transform.translation.y - transform1.transform.translation.y < -0.001):
			vel_msg.linear.y = -0.1
			pub.publish(vel_msg)
		elif (abs(transform.transform.translation.y - transform1.transform.translation.y) < 0.1):
			vel_msg.linear.y = 0
			pub.publish(vel_msg)
  
		if (transform.transform.translation.x - transform1.transform.translation.x > 0.001):
			print("linear.x")
			vel_msg.linear.x = 0.1
			pub.publish(vel_msg)
		elif (transform.transform.translation.x - transform1.transform.translation.x < -0.001):
			vel_msg.linear.x = -0.1
			pub.publish(vel_msg)
		elif (abs(transform.transform.translation.x - transform1.transform.translation.x) <= 0.1):
			vel_msg.linear.x = 0
			pub.publish(vel_msg)
		
		rospy.sleep(0.1)

rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)

rospy.spin()