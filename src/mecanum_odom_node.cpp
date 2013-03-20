#include "mecanum_drive/mecanum_odom.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mecanum_odom");
	ros::NodeHandle n;
  
	mecanum_drive::MecanumOdom profile(n);

	ros::spin();

	return 0;
}
