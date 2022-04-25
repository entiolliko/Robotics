#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>

#include "data.h"

#include <vector>


class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber wheel_states_listener;
	ros::Publisher vel_publisher;

	float wheel_radius;
	float w;
	float l;

	//create a message to read both velocity and position vectors
	sensor_msgs::JointState wheels;
	geometry_msgs::TwistStamped out_msg;
	bool flag; //Forse è da togliere se riusciamo a sincronizzare i messaggi.


public :
	node1(){
		this->wheel_states_listener = this-> n.subscribe("/wheel_states",1000,&node1::wheel_statesCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);

		flag = false;

		//robot parameters we can move somewhere else
		wheel_radius = 0.07;
		w = 0.169;
		l = 0.200;
		//inizialize our vectors
		for(int i=0; i<4;i ++){
			this->wheels.velocity.push_back(0.0);
			this->wheels.position.push_back(0.0);
		}

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

		for(int i = 0; i < 4; i++){
			wheels.velocity[i]= in_msg->velocity[i];
			wheels.position[i]= in_msg->position[i];
		}

		calculate_data();
	}


	//started with the formulas, they are not correct they need some factor multiplying
	void calculate_data()
	{
		double vx = (wheel_radius/4)*(wheels.velocity[0] + wheels.velocity[1] + wheels.velocity[2] + wheels.velocity[3]);
		double vy = (wheel_radius/4)*(-wheels.velocity[0] + wheels.velocity[1] + wheels.velocity[2] - wheels.velocity[3]);
		double omega  = (wheel_radius/4)*(1/(w+l))*(- wheels.velocity[0] + wheels.velocity[1] - wheels.velocity[2] + wheels.velocity[3]);

		out_msg.header.frame_id = "";
		out_msg.header.stamp = ros::Time::now();

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
