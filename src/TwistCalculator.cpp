#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <typeinfo>
#include "data.h"

class TwistCalculator{

private :
	ros::NodeHandle n;
	ros::Subscriber wheel_states_listener;
	ros::Publisher vel_publisher;

	//create a message to read both velocity and position vectors
	geometry_msgs::TwistStamped out_msg;

	bool start = true; //TODO: Forse è da togliere se riusciamo a sincronizzare i messaggi.


	double w1_old, w2_old, w3_old, w4_old;
	double vx, vy, omega;

	double w1_ok, w2_ok, w3_ok, w4_ok;
	double radpertick;
	ros::Time t_old, t_new;
	double dt;

public :
	TwistCalculator(){
		this->wheel_states_listener = this-> n.subscribe("/wheel_states",10000,&TwistCalculator::wheel_statesCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);

		vx = 0.0;
	  vy = 0.0;
	  omega = 0.0;
	  radpertick = (2*M_PI)/(CPR*GEAR_RATIO);
	  w1_ok = 0.0;
	  w2_ok = 0.0;
	  w2_ok = 0.0;
	  w4_ok = 0.0;
	  t_old = ros::Time::now();
	  t_new = ros::Time::now();

	}


	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			ros::spinOnce();
		}
	}

	// void calculate_speed_with_formulas(){
	// 	double vx = (R/4)*(in_msg->velocity[0] + in_msg->velocity[1] + in_msg->;velocity[2] + in_msg->velocity[3]) * (1/60.0) * (1/GEAR_RATIO);
	// 	double vy = (R/4)*(-in_msg->velocity[0] + in_msg->velocity[1] + in_msg->velocity[2] - in_msg->velocity[3]) * (1/60.0) * (1/GEAR_RATIO);
	// 	double omega  = (R/4)*(1/(W+L))*(- in_msg->velocity[0] + in_msg->velocity[1] - in_msg->velocity[2] + in_msg->velocity[3]) * (1/60.0) * (1/GEAR_RATIO);
	//
	// 	out_msg.header.seq = in_msg->header.seq;
	// 	out_msg.header.stamp = in_msg->header.stamp;
	// 	out_msg.header.frame_id = in_msg->header.frame_id;
	//
	// 	out_msg.twist.linear = toVector3(vx, vy, 0);
	// 	out_msg.twist.angular = toVector3(0, 0, omega);
	//
	// 	ROS_INFO("Ho pubblicato Questi Dati");
	// 	ROS_INFO("linear vale %f", out_msg.twist.linear.x);
	// 	ROS_INFO("linear vale %f", out_msg.twist.linear.y);
	// 	ROS_INFO("angular vale %f", out_msg.twist.angular.z);
	//
	// 	this->vel_publisher.publish(out_msg);
	// }

	void wheel_statesCallback(const sensor_msgs::JointState::ConstPtr& in_msg){
		//TODO: Implement the calculation of the wheel speeds from the ticks.

		if(!start)
		{
			t_new = in_msg->header.stamp;
			unsigned long delta = t_new.toNSec() - t_old.toNSec();
			dt =(double) delta / 1000000000.0;


			//ROS_INFO("La differenza su ruota 1 vale %f:",curr_w1);

			w1_ok = (in_msg->position[0] - w1_old)*radpertick/dt;
			w2_ok = (in_msg->position[1] - w2_old)*radpertick/dt;
			w3_ok = (in_msg->position[2] - w3_old)*radpertick/dt;
			w4_ok = (in_msg->position[3] - w4_old)*radpertick/dt;

			//ROS_INFO("w1 vale %f:",w1_ok);

			vx = (R/4)*(w1_ok + w2_ok + w3_ok + w4_ok);
			vy = (R/4)*(-w1_ok + w2_ok + w3_ok - w4_ok);
			omega  = (R/4)*(1/(W+L))*(- w1_ok + w2_ok - w3_ok + w4_ok);

			t_old = t_new;

		}
		else if(start){
			t_old = in_msg->header.stamp;
			start = false;
		}

	  out_msg.header.seq = in_msg->header.seq;
	  out_msg.header.stamp = in_msg->header.stamp;
	  out_msg.header.frame_id = in_msg->header.frame_id;

	  out_msg.twist.linear = toVector3(vx, vy, 0);
	  out_msg.twist.angular = toVector3(0, 0, omega);


	  w1_old = in_msg->position[0];
	  w2_old = in_msg->position[1];
	  w3_old = in_msg->position[2];
	  w4_old = in_msg->position[3];


	  this->vel_publisher.publish(out_msg);
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
	ros::init(argc,argv,"TwistCalculator");

	TwistCalculator test_node;
	test_node.main_loop();
}
