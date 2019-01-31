#include <ros/ros.h>
#include <setpoint_control.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_ctrl_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);
	SetpointCtrl ctrl(nh);
	ROS_INFO("setpoint_ctrl_node start");

	while (ros::ok())
	{
		ctrl.run();
		//printf("sending\n");

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}