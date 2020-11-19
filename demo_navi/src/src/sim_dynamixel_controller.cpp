#include <iostream> 
#include "include/sim_dynamixel_controller.h"
#include <cmath>

#define wheel_seperation 0.25
#define wheel_radius 0.035
#define pi 3.14159265359

using namespace std;

DynamixelController::DynamixelController(){}
DynamixelController::~DynamixelController(){}


double DynamixelController::jointPublisher(std_msgs::Float64 l,std_msgs::Float64 r,std_msgs::Float64 b){

    joint_vel_right_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/right_joint_velocity_controller/command", 1);
    joint_vel_left_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/left_joint_velocity_controller/command", 1);
    joint_vel_back_pub = priv_node_handle.advertise<std_msgs::Float64>("/demo_DIR/back_joint_velocity_controller/command", 1);

    joint_vel_right_pub.publish(r);
    joint_vel_left_pub.publish(l);
    joint_vel_back_pub.publish(b);
    cout<<"back : "<<b<<endl;
}

void DynamixelController::initSubscriber(){

    cmd_vel_sub = priv_node_handle.subscribe("/FN/cmd_vel",10, &DynamixelController::commandVelocityCallback,this);
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg){

    std_msgs::Float64 wll,wrr,wbb;

    double robot_linX_vel = msg -> linear.x;
    double robot_linY_vel = msg -> linear.y;
    double robot_ang_vel = msg -> angular.z;

    wll.data = ((robot_linX_vel * cos(2*pi/3)) - (sin(2*pi/3) * robot_linY_vel ) + (wheel_seperation * robot_ang_vel))/wheel_radius;
    wrr.data = ((robot_linX_vel * cos(-2*pi/3)) - (sin(-2*pi/3) * robot_linY_vel ) + (wheel_seperation * robot_ang_vel))/wheel_radius;
    wbb.data = (robot_linX_vel + (wheel_seperation * robot_ang_vel))/wheel_radius;  

    cout<<"back : "<<wbb<<endl;

    DynamixelController::jointPublisher(wll, wrr, wbb);

}

int main(int argc, char **argv){

    ros::init(argc, argv, "sim_dynamixel_controller");
    ros::NodeHandle node_handle("");

    //std_msgs::Float64 l = 1;
    //std_msgs::Float64 r = 1;
    //std_msgs::Float64 b = 1;

    DynamixelController dynamixel_controller;

    dynamixel_controller.initSubscriber();
    //dynamixel_controller.jointPublisher(l,r,b);

    ros::spin();

    return 0;
    
}