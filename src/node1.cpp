#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <vector>




enum integrationMethod{
	Euler, RK
};



class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener;
	ros::Subscriber wheel_states_listener;
	ros::Publisher vel_publisher;

	std::vector<std_msgs::Float64> position;
	std::vector<std_msgs::Float64> orientation;

	std_msgs::Float64 wheel_position[4];
	std_msgs::Float64 wheel_velocity[4];

	geometry_msgs::TwistStamped out_msg;

	bool flag;

public :
	node1(){
		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->wheel_states_listener = this-> n.subscribe("/wheel_states",1000,&node1::wheel_statesCallback,this);
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

			wheel_position[i].data = zeroInF.data;
			wheel_velocity[i].data = zeroInF.data;
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
		ROS_INFO("I heard [%f]",in_msg->pose.position.x);
		this->position[0].data = in_msg->pose.position.x;
		this->position[1].data = in_msg->pose.position.y;
		this->position[2].data = in_msg->pose.position.z;
    	this->orientation[0].data = in_msg->pose.orientation.x;
		this->orientation[1].data = in_msg->pose.orientation.y;
    	this->orientation[2].data = in_msg->pose.orientation.z;
		this->orientation[3].data = in_msg->pose.orientation.w;
		//Qua dobbiamo capire un po cosa fare con la posizione e la orientazione
	}

	void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& in_msg){

		for(int i = 0; i < 4; i++){
			this->wheel_position[i].data = in_msg->position[i];
			this->wheel_velocity[i].data = in_msg->velocity[i];
		}
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
