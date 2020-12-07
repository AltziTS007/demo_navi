#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <demo_navi/DynamixelStateList.h>
#include <demo_navi/DynamixelState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


class DynamixelController{

    private:
        //ROS NodeHnadle
        ros::NodeHandle node_handle;
        ros::NodeHandle priv_node_handle;

        //ROS Topc Publisher
        ros::Publisher joint_vel_right_pub;
        ros::Publisher joint_vel_left_pub;
        ros::Publisher joint_vel_back_pub;
	ros::Publisher state_pub;

        //ROS Topic Subscriber
        ros::Subscriber cmd_vel_sub;
        
    public:
        DynamixelController();
        ~DynamixelController();
        
        double jointPublisher(std_msgs::Float64 l, std_msgs::Float64 r, std_msgs::Float64 b);
        void initSubscriber(void);

        void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
	void commandSateCallback(const std_msgs::Float64::ConstPtr &flt_msg);       
};
