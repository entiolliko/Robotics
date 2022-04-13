#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Float64.h>
#include <vector>


class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener ;
	ros::Publisher vel_publisher;

	std::vector<std_msgs::Float64> position;
	std::vector<std_msgs::Float64> orientation;

	std::vector<std_msgs::Float64> linear_velocity;
	std::vector<std_msgs::Float64> angular_velocity;

	geometry_msgs::TwistStamped out_msg;
	bool flag;

public :
	node1(){
		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);

		for(int i = 0; i < 3; i++){
			std_msgs::Float64 zeroInF;
			zeroInF.data = 0.0;
			this->position.push_back(zeroInF);
		}

		for(int i = 0; i < 4; i++){
			std_msgs::Float64 zeroInF;
			zeroInF.data = 0.0;
			this->orientation.push_back(zeroInF);
		}

		for(int i = 0; i < 3; i++){
			std_msgs::Float64 zeroInF;
			zeroInF.data = 0.0;
			this->linear_velocity.push_back(zeroInF);
		}

		for(int i = 0; i < 3; i++){
			std_msgs::Float64 zeroInF;
			zeroInF.data = 0.0;
			this->angular_velocity.push_back(zeroInF);
		}

		flag = false;
		}

	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			if(flag){
				this->vel_publisher.publish(out_msg);
				this->flag = false;
			}
			ros::spinOnce();
		}
	}

	void  wheelCallback(){

	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& in_msg){
		//ROS_INFO("I heard [%f]",msg->pose.position.x);
		this->linear_velocity[0].data = in_msg->pose.position.x;
		this->linear_velocity[1].data = in_msg->pose.position.y;
		this->linear_velocity[2].data = in_msg->pose.position.z;
    this->angular_velocity[0].data = in_msg->pose.orientation.x;
		this->angular_velocity[1].data = in_msg->pose.orientation.y;
    this->angular_velocity[2].data = in_msg->pose.orientation.z;
		this->angular_velocity[3].data = in_msg->pose.orientation.w;
		calculate_data();
	}

	void calculate_data()
	{
		this-> flag = true;
	}

	void calculate_euler(){

	}

	void calculate_rk(){


	}
};

int main(int argc, char** argv){
	ros::init(argc,argv,"node1");

	node1 test_node;
	test_node.main_loop();
}
