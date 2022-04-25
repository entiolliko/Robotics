#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <vector>

#include "data.h"

class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber wheel_states_listener;
	ros::Publisher vel_publisher;

	//create a message to read both velocity and position vectors
	geometry_msgs::TwistStamped out_msg;

	bool flag; //TODO: Forse è da togliere se riusciamo a sincronizzare i messaggi.


public :
	node1(){
		this->wheel_states_listener = this-> n.subscribe("/wheel_states",1000,&node1::wheel_statesCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);

		flag = false;

	}


	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			if(flag){
				this->vel_publisher.publish(out_msg);
				ROS_INFO("Ho pubblicato Questi Dati");
			  ROS_INFO("linear vale %f",out_msg.twist.linear.x);
			  ROS_INFO("linear vale %f",out_msg.twist.linear.y);
				ROS_INFO("angular vale %f",out_msg.twist.angular.z);

				this->flag = false;
			}
			ros::spinOnce();
		}
	}

	void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& in_msg){
		double vx = (R/4)*(in_msg->velocity[0] + in_msg->velocity[1] + in_msg->velocity[2] + in_msg->velocity[3]) * (1/60) * (1/GEAR_RATIO);
		double vy = (R/4)*(-in_msg->velocity[0] + in_msg->velocity[1] + in_msg->velocity[2] - in_msg->velocity[3]) * (1/60) * (1/GEAR_RATIO);
		double omega  = (R/4)*(1/(W+L))*(- in_msg->velocity[0] + in_msg->velocity[1] - in_msg->velocity[2] + in_msg->velocity[3]) * (1/60) * (1/GEAR_RATIO);

		out_msg.header.seq = in_msg->header.seq;
		out_msg.header.stamp = in_msg->header.stamp;
		out_msg.header.frame_id = in_msg->header.frame_id;

		out_msg.twist.linear = toVector3(vx, vy, 0);
		out_msg.twist.angular = toVector3(0, 0, omega);


		this-> flag = true;
	}


	//this function returns three values inside a Vector3 type
	geometry_msgs::Vector3 toVector3(double a,double b, double c){
		geometry_msgs::Vector3 tmp;
		tmp.x = a;
		tmp.y = b;
		tmp.z = c;
		return tmp;
	}
};

int main(int argc, char** argv){
	ros::init(argc,argv,"node1");

	node1 test_node;
	test_node.main_loop();
}
