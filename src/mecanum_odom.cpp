#include "mecanum_drive/mecanum_odom.h"

namespace mecanum_drive
{

void MecanumOdom::statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	double dx;
	double dy;
	double dth;

	double radFrontLeft = 0;
	double radFrontRight = 0;
	double radBackRight = 0;
	double radBackLeft = 0;
	//do the kinematics here (jointState --> vx, vy, vth)
	unsigned int i;

	for(i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == leftFrontJointName)
		{
			radFrontLeft = msg->position[i] - last_lf;
			last_lf = msg->position[i];
		}
		else if(msg->name[i] == rightFrontJointName)
		{
			radFrontRight = msg->position[i] - last_rf;
			last_rf = msg->position[i];
		}
		else if(msg->name[i] == rightBackJointName)
		{
			radBackRight = msg->position[i] - last_rb;
			last_rb = msg->position[i];
		}
		else if(msg->name[i] == leftBackJointName)
		{
			radBackLeft = msg->position[i] - last_lb;
			last_lb = msg->position[i];
		}
		else
		{
			ROS_WARN( "Received unknown wheel state %s", msg->name[i].c_str( ) );
		}
	}
	
	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the four wheels
	double dt = (current_time - last_time).toSec();
	if (dt == 0.0)
	{
		ROS_ERROR("dt = 0. This is bad.");
		return;
	}

	double inverseAvgLengths = wheelRadius / (2.0 * (axleLength + wheelbaseLength));
	
	dx = wheelRadius * 0.25 * (radFrontRight + radFrontLeft + radBackRight + radBackLeft);
	dy = wheelRadius * 0.25 * (radFrontRight - radFrontLeft - radBackRight + radBackLeft);
	dth = inverseAvgLengths * (-radFrontLeft - radBackLeft + radFrontRight + radBackRight);

	x += (dx*cos(th) - dy*sin(th));
	y += (dx*sin(th) + dy*cos(th));
	th += dth;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	//odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_footprint";
	odom.twist.twist.linear.x = dx / dt;
	odom.twist.twist.linear.y = dy / dt;
	odom.twist.twist.angular.z = dth / dt;

	odom.pose.covariance[0] = .1;
	odom.pose.covariance[7] = .1;
	odom.pose.covariance[14] = 1000000000;
	odom.pose.covariance[21] = 1000000000;
	odom.pose.covariance[28] = 1000000000;
	odom.pose.covariance[35] = .1;

	odom.twist.covariance[0] = .1;
	odom.twist.covariance[7] = .1;
	odom.twist.covariance[14] = 1000000000;
	odom.twist.covariance[21] = 1000000000;
	odom.twist.covariance[28] = 1000000000;
	odom.twist.covariance[35] = .1;

	//publish the message
	odom_pub.publish(odom);

	last_time = current_time;
} 

MecanumOdom::MecanumOdom(ros::NodeHandle &n) : n(n)
{
	ros::NodeHandle private_nh("~");
	
	private_nh.param<double>("wheel_radius", wheelRadius, 1.0);
	private_nh.param<double>("axle_length", axleLength, 1.0);
	private_nh.param<double>("wheelbase_length", wheelbaseLength, 1.0);
	private_nh.param<std::string>("left_front_joint_name", leftFrontJointName, "left_front_wheel_joint");
	private_nh.param<std::string>("right_front_joint_name", rightFrontJointName, "right_front_wheel_joint");
	private_nh.param<std::string>("left_back_joint_name", leftBackJointName, "left_back_wheel_joint");
	private_nh.param<std::string>("right_back_joint_name", rightBackJointName, "right_back_wheel_joint");

	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	joint_states = n.subscribe<sensor_msgs::JointState>("joint_states", 1, &MecanumOdom::statesCallback, this);

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	last_rf = 0;
	last_lf = 0;
	last_rb = 0;
	last_lb = 0;
	x = 0.0;
	y = 0.0;
	th = 0.0;
}

MecanumOdom::~MecanumOdom()
{
	odom_pub.shutdown();
	joint_states.shutdown();
}

double MecanumOdom::getWheelRadius()
{
	return MecanumOdom::wheelRadius;
}
	
double MecanumOdom::getAxleLength()
{
	return MecanumOdom::axleLength;
}

double MecanumOdom::getWheelbaseLength()
{
	return wheelbaseLength;
}

void MecanumOdom::setWheelRadius(double radius)
{
	if(radius > 0.0)
		wheelRadius = radius;
}

void MecanumOdom::setAxleLength(double length)
{
	if(length > 0.0)
		axleLength = length;
}

void MecanumOdom::setWheelbaseLength(double length)
{
	if(length > 0.0)
		wheelbaseLength = length;
}

};
