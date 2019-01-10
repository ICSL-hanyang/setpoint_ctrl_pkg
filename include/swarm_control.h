#ifndef SWARM_CONTROL_H
#define SWARM_CONTROL_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

static unsigned int my_id;

class VehiclePos
{
private:
  unsigned int id;
  tf2::Vector3 relative_pos;

public:
  VehiclePos();
  VehiclePos(const unsigned int&);

  const unsigned int getID() const;
  void setRelativePos(const tf2::Vector3&);
  tf2::Vector3 getRelativePos() const;
};

class SwarmCtrl
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_global;
  
  /* 현재 모드가 offboard 모드일때만 업데이트를 하기 위해 필요 */
  ros::Subscriber state_sub;
  mavros_msgs::State cur_state;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

  unsigned int num_drone;

  std::vector<VehiclePos> vehicle_positions;

  tf2::Vector3 position;
  tf2::Vector3 velocity;
  tf2::Vector3 acceleration;

  double m;
  double kp;
  double kp_s;
  double range;

  double max_force;
  double max_speed;
  double seek_weight;
  double separate_weight;
  void limit(tf2::Vector3, float);

  void stateCB(const mavros_msgs::State::ConstPtr &msg);
  tf2::Vector3 cohesion(); /* 미구현 */

public:
  SwarmCtrl(tf2::Vector3);
  SwarmCtrl(SwarmCtrl &&) = default;
  SwarmCtrl(const SwarmCtrl &) = default;
  SwarmCtrl &operator=(SwarmCtrl &&) = default;
  SwarmCtrl &operator=(const SwarmCtrl &) = default;
  ~SwarmCtrl();

  void getNeighborPos();
  tf2::Vector3 seek();
  tf2::Vector3 separate();
  void update();
  void transformSender();
  void run();
};

#endif