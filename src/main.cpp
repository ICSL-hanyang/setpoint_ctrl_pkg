#include <ros/ros.h>
#include <swarm_control.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_ctrl_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);
	SetpointCtrl ctrl(nh);
	ROS_INFO("setpoint_ctrl_node start");

	while (ros::ok())
	{
		ctrl.run();

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}