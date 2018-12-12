#include <ros/ros.h>
#include <swarm_control.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "setpoint_ctrl_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(30);
	SwarmCtrl ctrl(tf2::Vector3(3, 3, 3));
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