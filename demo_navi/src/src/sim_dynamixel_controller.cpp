#include <iostream> 
#include "include/sim_dynamixel_controller.h"
#include <cmath>
#include <demo_navi/DynamixelStateList.h>
#include <demo_navi/DynamixelState.h>

#define wheel_seperation 0.25
#define wheel_radius 0.038
#define pi 3.14159265359
#define AMPlIFY 100
#define LEFT 0
#define RIGHT 1
#define BACK 2

using namespace std;

DynamixelController::DynamixelController(){}
DynamixelController::~DynamixelController(){}


double DynamixelController::jointPublisher(std_msgs::Float64 l,std_msgs::Float64 r,std_msgs::Float64 b){

    joint_vel_right_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/right_joint_velocity_controller/command", 1);
    joint_vel_left_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/left_joint_velocity_controller/command", 1);
    joint_vel_back_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/back_joint_velocity_controller/command", 1);
    state_pub = priv_node_handle.advertise<demo_navi::DynamixelStateList>("/demo_DIR/StatePublisher", 1);

    joint_vel_right_pub.publish(r);
    joint_vel_left_pub.publish(l);
    joint_vel_back_pub.publish(b);
 	
    demo_navi::DynamixelStateList dynamixel_state_list;
 	demo_navi::DynamixelState dynamixel_state[3];
 	dynamixel_state_list.dynamixel_state.clear();

 	dynamixel_state[LEFT].present_velocity = l.data;
  	dynamixel_state[RIGHT].present_velocity = r.data;
 	dynamixel_state[BACK].present_velocity = b.data;

 	for(uint8_t index = 0; index < 3; index++)
 	dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);


    state_pub.publish(dynamixel_state_list);
}

void DynamixelController::initSubscriber(){

    cmd_vel_sub = priv_node_handle.subscribe("/ep/cmd_vel",10, &DynamixelController::commandVelocityCallback,this);
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg){

    std_msgs::Float64 wll,wrr,wbb;

    double robot_linX_vel = msg -> linear.x;
    double robot_linY_vel = msg -> linear.y;
    double robot_ang_vel = msg -> angular.z;

    wll.data = (((robot_linY_vel * cos(2*pi/3)) - (sin(2*pi/3) * robot_linX_vel ) + (wheel_seperation * robot_ang_vel))/wheel_radius) * AMPlIFY;	//rad/s wheel_seperation = -0.35
    wrr.data = (((robot_linY_vel * cos(-2*pi/3)) - (sin(-2*pi/3) * robot_linX_vel ) + (wheel_seperation * robot_ang_vel))/wheel_radius) * AMPlIFY;	//rad/s  wheel_seperation = 0.35
    wbb.data = ((robot_linY_vel + (wheel_seperation * robot_ang_vel))/wheel_radius) * AMPlIFY;	//rad wheel_seperation = 0.25


    DynamixelController::jointPublisher(wll, wrr, wbb);


}

int main(int argc, char **argv){

    ros::init(argc, argv, "sim_dynamixel_controller");
    ros::NodeHandle node_handle("");

    DynamixelController dynamixel_controller;

    dynamixel_controller.initSubscriber();

    ros::spin();

    return 0;
    
}
