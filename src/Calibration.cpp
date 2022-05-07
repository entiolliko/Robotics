#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <sstream>
#include "project1/custom.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Calibration {

private:
    ros::NodeHandle n; //The handler of the current node

    ros::Subscriber sub_odom;
    ros::Subscriber sub_gt;

    ros::Publisher custom_odom_pub;
    ros::Publisher custom_gt_pub;

    project1::custom out_msgs1;
    project1::custom out_msgs2;

public:
    Calibration() {
        sub_odom = n.subscribe("/odom", 1000, &Calibration::odometryCallback, this);
        custom_odom_pub = n.advertise<project1::custom>("/customOdom", 1000);

        sub_gt = n.subscribe("/robot/pose", 1000, &Calibration::gtCallback, this);
        custom_gt_pub = n.advertise<project1::custom>("/customGT", 1000);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
      out_msgs1.x = msg->pose.pose.position.x;
      out_msgs1.y = msg->pose.pose.position.y;

      tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      out_msgs1.th = yaw;

      custom_odom_pub.publish(out_msgs1);

    }

    void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      out_msgs2.x = msg->pose.position.x;
      out_msgs2.y = msg->pose.position.y;

      tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      out_msgs2.th = yaw;

      custom_gt_pub.publish(out_msgs2);

    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Calibration");
    Calibration pubSubOdometry;

    ros::spin();
    return 0;
}
