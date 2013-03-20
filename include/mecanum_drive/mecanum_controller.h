#ifndef __MECANUM_CONTROLLER__H__
#define __MECANUM_CONTROLLER__H__

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include <string.h>

namespace mecanum_drive
{

class MecanumController
{
	public:
		MecanumController(ros::NodeHandle &n);
		~MecanumController();
		void callback(const geometry_msgs::Twist::ConstPtr &msg);

	private:
		ros::NodeHandle n;
		ros::Publisher jointPub;
		ros::Subscriber sub;
		double wheelBase1;
		double wheelBase2;
		double wheelRadius;
		std::string frontLeft;
		std::string frontRight;
		std::string backLeft;
		std::string backRight;
		void inverseMecanum4WKinematics(const double &linear_x, const double &linear_y, const double &angular_z);
};

};


#endif
