#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64.h>
#include <vector>


//dynamic reconfigure
#include <project1/parametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>

enum integrationMethod{
	Euler, RK
};
//-----------------------------



class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener;
	ros::Subscriber wheel_states_listener;
	ros::Publisher vel_publisher;

	std::vector<std_msgs::Float64> position;
	std::vector<std_msgs::Float64> orientation;


	//create a message to read both velocity and position vectors
	sensor_msgs::JointState wheels;
	float wheel_radius;
	float w;
	float l;


	geometry_msgs::TwistStamped out_msg;

	bool flag;

	//-------------------------------------------
	integrationMethod mode;
	//dynamic reconfigure server declaration
	dynamic_reconfigure::Server<project1::parametersConfig> method_server;
	dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType method_callback;
	//---------------------------------------


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
		}

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

		//dynamic reconfigure --------------------
		mode = Euler;
		method_callback = boost::bind(&node1::setMode, this, _1, _2);
        method_server.setCallback(method_callback);
        //----------------------------------------
		}


	//this function sets the mode to Euler or RK
	//we need to add the second parameter to this function also
	void setMode(project1::parametersConfig &config, uint32_t level){
		switch(config.odomMethod){
			case 0: mode = Euler; break;
			case 1: mode = RK; break;
			//default: break;
		}
	}//-------------------------------------------------------------------



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

	void  wheelCallback(){

	}

	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& in_msg){
		//ROS_INFO("I heard [%f]",in_msg->pose.position.x);
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
			wheels.velocity[i]= in_msg->velocity[i];
			wheels.position[i]= in_msg->position[i];
		}

		calculate_data();
	}


	//started with the formulas, they are not correct they need some factor multiplying
	void calculate_data()
	{
		double vx = (wheel_radius/4)*(wheels.position[0] + wheels.position[1] + wheels.position[2] + wheels.position[3]);
		double vy = (wheel_radius/4)*(-wheels.position[0] + wheels.position[1] + wheels.position[2] - wheels.position[3]);
		double omega  = (wheel_radius/4)*(1/(w+l))*(- wheels.position[0] + wheels.position[1] - wheels.position[2] + wheels.position[3]);
		
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