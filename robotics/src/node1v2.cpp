#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <vector>
using namespace std;

class node1{

private :
	ros::NodeHandle n;
	ros::Subscriber pose_listener;
	ros::Publisher vel_publisher;

	std::vector<float64> linear_velocity;
	std::std::vector<float64> angular_velocity;

	geometry_msgs::TwistStamped out_msg;
	bool readFromBag;

public :
	node1(){
		//TODO: Perché hai usato un puntatore per il poseCallback e perché hai messo this?
		this->pose_listener = this-> n.subscribe("/robot/pose",1000,&node1::poseCallback,this);
		this->vel_publisher = this-> n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
		this->linear_velocity.insert(linear_velocity.begin(), 3, 0);
		this->angular_velocity.insert(angular_velocity.begin(), 4, 0);
		this->readFromBag = false;
		}

	void main_loop(){
		ros::Rate loop_rate(10);

		while(ros::ok()){
			if(readFromBag){
				this->vel_publisher.publish(out_msg);
				this->readFromBag = false;
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

	//TODO: Fare l'implementazione delle formule in tick
	void calculate_data()
	{

		this-> readFromBag = true;
	}

	//TODO: Fare l'implementazione dell'approssimazione di Euler
	void calculate_euler(){

	}

	//TODO: Fare l'implementazione dell'approssimazione di Runga-Tunga
	void calculate_rk(){


	}
};

int main(int argc, char** argv){
	ros::init(argc,argv,"node1");

	node1 test_node;
	test_node.main_loop();
}
