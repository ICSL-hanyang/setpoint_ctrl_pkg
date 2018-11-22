#include <ros/ros.h>
#include <swarm_control.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_ctrl_node");
	ros::NodeHandle nh("~");

    SwarmCtrl ctrl(tf2::Vector3(3, 3, 3));
	
	return 0;
}