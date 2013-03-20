#ifndef MECANUM_ODOM_H
#define MECANUM_ODOM_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

namespace mecanum_drive
{

class MecanumOdom
{
	public:
		MecanumOdom(ros::NodeHandle &n);
		~MecanumOdom();
		double getWheelRadius();
		double getAxleLength();
		double getWheelbaseLength();
		void setWheelRadius(double radius); //in m
		void setAxleLength(double length); //in m
		void setWheelbaseLength(double length); //in m
  
	private:
		void statesCallback(const sensor_msgs::JointState::ConstPtr& msg);

		double axleLength;
		//NOTE: Axle Length is from the left wheels to the right wheels
		double wheelRadius;
		//NOTE: Wheelbase Length is from the rear wheels to the front wheels
		double wheelbaseLength;

		ros::Publisher odom_pub;
		tf::TransformBroadcaster odom_broadcaster;
		ros::NodeHandle n;
		ros::Subscriber joint_states;
		ros::Time current_time, last_time;
		std::string rightFrontJointName;
		std::string leftFrontJointName;
		std::string rightBackJointName;
		std::string leftBackJointName;

		double last_rf;
		double last_lf;
		double last_rb;
		double last_lb;
		double x;
		double y;
		double th;
};

};

#endif
