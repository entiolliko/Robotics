#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JoinState.h"
#include <vector>


class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener ;
	ros::Publisher vel_publisher;

	std::vector<float64> position;
	std::vector<float64> orientation;

	std::vector<float64> linear_velocity;
	std::std::vector<float64> angular_velocity;

	geometry_msgs::TwistStamped out_msg;
	bool flag;

public :
	node1(){
		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs/TwistStamped>("cmd_vel",1000);

		this->position.insert(position.begin(), 3, 0);
		this->orientation.insert(orientation.begin(), 4,0);

		this->linear_velocity.insert(linear_velocity.begin(), 3, 0);
		this->angular_velocity.insert(angular_velocity.begin(), 3,0);
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
		this->linear_velocity[0] = in_msg->pose.position.x;
		this->linear_velocity[1] = in_msg->pose.position.y;
		this->linear_velocity[2] = in_msg->pose.position.z;
    this->angular_velocity[0] = in_msg->pose.orientation.x;
		this->angular_velocity[1] = in_msg->pose.orientation.y;
    this->angular_velocity[2] = in_msg->pose.orientation.z;
		this->angular_velocity[3] = in_msg->pose.orientation.w;
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
