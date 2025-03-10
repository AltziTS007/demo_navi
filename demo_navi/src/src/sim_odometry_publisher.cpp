#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <demo_navi/DynamixelStateList.h>
#include <demo_navi/DynamixelState.h>
#include <cmath>
#define wheel_seperation 0.25
#define wheel_radius 0.038

double Vll = 0.0;
double Vrr = 0.0;
double Vbb = 0.0;



double convertValue2Velocity(const double &value)
{
    double velocity = 0;
    const double RPM2RADPERSEC = 0.104719755f;
    const float RPM = 0.732f;
    const float coefficient = 0.012f;
    velocity = value * RPM * RPM2RADPERSEC * wheel_radius * coefficient;
    return velocity;  //m/s
}

double throwV(double V){
  return V;
}

void DynamixelStateCallback(const demo_navi::DynamixelStateListConstPtr& msg){

    Vll = convertValue2Velocity(msg-> dynamixel_state[0].present_velocity);
    Vrr = convertValue2Velocity(msg-> dynamixel_state[1].present_velocity);
    Vbb = convertValue2Velocity(msg-> dynamixel_state[2].present_velocity);

    //ROS_INFO("01Vl: [%lf], 01Vr: [%lf]", Vll,Vrr);
}




int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  //ROS_INFO("Vl: [%lf], Vr: [%lf]", Vll,Vrr);
  ros::Subscriber dynamixel_state_sub = n.subscribe("/demo_DIR/StatePublisher", 1, DynamixelStateCallback);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    double vy = ((2*Vbb - Vll - Vrr) / 3.0);
    double vx = ((sqrt(3.0)*Vrr - sqrt(3.0)*Vll) / 3.0);
    double vth = ((Vrr + Vbb + Vll) / (3.0*wheel_seperation));

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}