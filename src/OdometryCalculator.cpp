#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include "project1/parametersConfig.h"
#include "project1/Reset.h"

class OdometryCalculator {

private:
    ros::NodeHandle n; //The handler of the current node
    ros::Subscriber sub; // The subscripter to the /cmd_vel topic
    ros::Publisher odom_pub;
    ros::ServiceServer resetPoseService;

    ros::Time lastTime;
    double x = 0.0;
		double y = 0.0;
		double th = 0.0;
    tf::TransformBroadcaster odom_broadcaster;

    int integrationType = 0;


public:
    OdometryCalculator() {
        sub = n.subscribe("/cmd_vel", 1000, &OdometryCalculator::computeOdometry, this);
        odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

        resetPoseService = n.advertiseService("resetposeService" , &OdometryCalculator::resetPose, this);

        lastTime = ros::Time::now();

    }

    void computeOdometry(const geometry_msgs::TwistStamped::ConstPtr& msg){
        double vx, vy, w, dt;
        ros::Time currentTime;

        //Reads currentTime from message's header
        currentTime = msg->header.stamp;

        unsigned long Ts_NSec =  currentTime.toNSec() - lastTime.toNSec();
        dt = (double)Ts_NSec / 1000000000.0;
        lastTime = currentTime;

        //Computes dt from last message
        //dt = (currentTime - lastTime).toSec();

        //Computes reads linear and angular velocities from message
        vx = msg->twist.linear.x;
        vy = msg->twist.linear.y;
        w = msg->twist.angular.z;

        //IntegrationintegrationType
        if(integrationType == 0) { //EULER
            x += vx * cos(th) * dt;
            y += vy * sin(th) * dt;
            th += w * dt;
        }
        else if(integrationType == 1){ //RUNGE-KUTTA
            x += vx * cos(th + w * dt / 2) * dt;
            y += vy * sin(th + w * dt / 2) * dt;
            th += w * dt;
        }

        //Publish tf transformation
        publishTfTransformation(currentTime);
        //Publish odometry message
        publishOdometry(vx, vy, w, currentTime);

    }

    void publishOdometry(double vx, double vy, double w, ros::Time currentTime){
        nav_msgs::Odometry odometry;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometry.header.stamp = currentTime;
        odometry.header.frame_id = "odom";
        //set pose
        odometry.pose.pose.position.x = x;
        odometry.pose.pose.position.y = y;
        odometry.pose.pose.position.z = 0.0;
        odometry.pose.pose.orientation = odometryQuaternion;
        //set velocity
        odometry.child_frame_id = "base_link";
        odometry.twist.twist.linear.x = vx;
        odometry.twist.twist.linear.y = vy;
        odometry.twist.twist.linear.z = 0;
        odometry.twist.twist.angular.x = 0;
        odometry.twist.twist.angular.y = 0;
        odometry.twist.twist.angular.z = w;

        //publish odometry
        odom_pub.publish(odometry);
    }

    void publishTfTransformation(ros::Time currentTime){
        geometry_msgs::TransformStamped odometryTransformation;
        geometry_msgs::Quaternion odometryQuaternion = tf::createQuaternionMsgFromYaw(th);

        //set header
        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_link";
        //set transformation
        odometryTransformation.transform.translation.x = x;
        odometryTransformation.transform.translation.y = y;
        odometryTransformation.transform.translation.z = 0;
        odometryTransformation.transform.rotation = odometryQuaternion;

        //publish transformation
        odom_broadcaster.sendTransform(odometryTransformation);
    }

    void setIntegration(project1::parametersConfig &config){
        integrationType = config.odomMethod;
    }

    bool resetPose(project1::Reset::Request  &req,
                   project1::Reset::Response &res)
    {
			x = req.reset_x;
			y = req.reset_y;
			th = req.reset_theta;
			ROS_INFO("x reset to: %f", x);
			ROS_INFO("y reset to: %f", y);
			ROS_INFO("th reset to: %f", th);
			return true;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "OdometryCalculator");
    OdometryCalculator pubSubOdometry;

    dynamic_reconfigure::Server<project1::parametersConfig> server;
    dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;

    f = boost::bind(&OdometryCalculator::setIntegration, &pubSubOdometry, _1);
    server.setCallback(f);

    ros::spin();
    return 0;
}
