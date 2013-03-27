#include "mecanum_drive/mecanum_controller.h"
#include "mecanum_drive/mecanum_controller.h"

namespace mecanum_drive
{

MecanumController::MecanumController(ros::NodeHandle &n) : n(n)
{
	ros::NodeHandle nh("~");
	nh.param<double>("wheelBase1", wheelBase1, 1.0);
	nh.param<double>("wheelBase2", wheelBase2, 1.0);
	nh.param<double>("wheel_radius", wheelRadius, 0.2);
	nh.param<std::string>("front_left", frontLeft, "front_left");
	nh.param<std::string>("front_right", frontRight, "front_right");
	nh.param<std::string>("back_left", backLeft, "back_left");
	nh.param<std::string>("back_right", backRight, "back_right");
	if (wheelBase1 <= 0)
	{
		ROS_WARN("wheelBase1 must be greater than zero");
		wheelBase1= 1.0;
	}
	if (wheelBase2 <= 0)
	{
		ROS_WARN("wheelBase2 must be greater than zero");
		wheelBase2= 1.0;
	}
	if (wheelRadius <= 0)
	{
		ROS_WARN("wheel_radius must be greater than zero");
		wheelRadius = 0.2;
	}

	sub = n.subscribe("cmd_vel",1, &MecanumController::callback, this);


	jointPub = n.advertise<trajectory_msgs::JointTrajectory>("cmd_joint_traj", 1);
}

MecanumController::~MecanumController()
{
	sub.shutdown();
	jointPub.shutdown();
}

void MecanumController::callback(const geometry_msgs::Twist::ConstPtr &msg)
{
	inverseMecanum4WKinematics(msg->linear.x, msg->linear.y, msg->angular.z);
}

void MecanumController::inverseMecanum4WKinematics(const double &linear_x, const double &linear_y, const double &angular_z)
{
	trajectory_msgs::JointTrajectory traj;
	trajectory_msgs::JointTrajectoryPoint point;
	point.velocities.resize(4);

	/// \todo I (Scott) found some different equations of motion, and they seem to work better (I optimized them a little, too)

	// temp
	const double temp1 = angular_z * ( wheelBase1 + wheelBase2 ) / 2.0;

	//const double c1 = 1.0 / (2.0 * M_PI * wheelRadius);
	//const double c2 = angular_z * (wheelBase1 + wheelBase2) / 2.0;

	// wheel 1:
	traj.joint_names.push_back(frontLeft);
	//point.velocities[0] = c1 * (linear_x - linear_y - c2);
	//point.velocities[0] = (1.0 / (wheelRadius * 2.0 * M_PI)) * ( linear_x - linear_y - (wheelBase1 + wheelBase2) / 2.0 * angular_z); 
	point.velocities[0] = ( linear_x - linear_y - temp1 ) / wheelRadius;

	// wheel 2:
	traj.joint_names.push_back(frontRight);
	//point.velocities[1] = c1 * (linear_x + linear_y + c2);
	//point.velocities[1] = (1.0 / (wheelRadius * 2.0 * M_PI)) * ( linear_x + linear_y + (wheelBase1 + wheelBase2) / 2.0 * angular_z); 
	point.velocities[1] = ( linear_x + linear_y + temp1 ) / wheelRadius;

	// wheel 3:
	traj.joint_names.push_back(backLeft);
	//point.velocities[2] = c1 * (linear_x + linear_y - c2);
	//point.velocities[2] = (1.0 / (wheelRadius * 2.0 * M_PI)) * ( linear_x + linear_y - (wheelBase1 + wheelBase2) / 2.0 * angular_z); 
	point.velocities[2] = ( linear_x + linear_y - temp1 ) / wheelRadius;

	// wheel 4:
	traj.joint_names.push_back(backRight);
	//point.velocities[3] = c1 * (linear_x - linear_y + c2);
	//point.velocities[3] = (1.0 / (wheelRadius * 2.0 * M_PI)) * ( linear_x - linear_y + (wheelBase1 + wheelBase2) / 2.0 * angular_z); 
	point.velocities[3] = ( linear_x - linear_y + temp1 ) / wheelRadius;

	traj.points.push_back(point);

	jointPub.publish(traj);
}

};
