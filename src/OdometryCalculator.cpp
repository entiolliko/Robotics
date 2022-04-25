#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include <project1/parametersConfig.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <vector>


class OdometryCalculator{

private :
	ros::NodeHandle n;
	ros::Subscriber robot_speed_listener;
	ros::Publisher odom_topic;

	nav_msgs::Odometry out_msg;

	bool flag;

public :
	OdometryCalculator()
	{
		this->robot_speed_listener = this-> n.subscribe("/cmd_vel",1000,&OdometryCalculator::robot_speed_listenerCallback,this);
		this->odom_topic = this -> n.advertise<nav_msgs::Odometry>("/odom",1000);

		flag = false;
	}



	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			if(flag){


			flag = false;
			}

		ros::spinOnce();
		}
	}

	void robot_speed_listenerCallback(const geometry_msgs::TwistStamped::ConstPtr& in_msg)
	{

	}

	void calculateEuler(){

	}

	void calculateRK(){

	}

};


int main(int argc, char** argv){
	ros::init(argc,argv,"OdometryCalculator");

	OdometryCalculator test_node;
	test_node.main_loop();
}
