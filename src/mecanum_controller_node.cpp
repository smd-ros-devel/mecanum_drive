#include "mecanum_drive/mecanum_controller.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "mecanum_controller");

	ros::NodeHandle n;

	mecanum_drive::MecanumController drive(n);

	ros::spin();

	return 0;
}
