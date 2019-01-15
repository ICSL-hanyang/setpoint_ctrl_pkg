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
  const unsigned int id;
  tf2::Vector3 relative_pos;

public:
  VehiclePos() = delete;
  VehiclePos(const unsigned int&);

  const unsigned int getID() const;
  void setRelativePos(const tf2::Vector3&);
  tf2::Vector3 getRelativePos() const;
};

class SetpointCtrl
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_global;
  
  /* 현재 모드가 offboard 모드일때만 업데이트를 하기 위해 필요 */
  ros::Subscriber state_sub;
  mavros_msgs::State cur_state;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

  std::vector<VehiclePos> vehicle_positions;

  tf2::Vector3 position;

  static unsigned int num_drone;
  static double kp;
  static double kp_s;
  static double range;
  static double max_speed;

  double max_force; // launch file paramer interface 때문에 남겨둠
  double seek_weight; // launch file paramer interface 때문에 남겨둠
  double separate_weight; // launch file paramer interface 때문에 남겨둠

  void limit(tf2::Vector3, double);
  void getNeighborPos();
  tf2::Vector3 separate();
  tf2::Vector3 seek();
  void update();
  void reset();
  void transformSender();
  tf2::Vector3 cohesion(); /* 미구현 */

  void stateCB(const mavros_msgs::State::ConstPtr &msg);

public:
  SetpointCtrl(tf2::Vector3);
  SetpointCtrl(SetpointCtrl &&) = default;
  SetpointCtrl(const SetpointCtrl &) = default;
  SetpointCtrl &operator=(SetpointCtrl &&) = default;
  SetpointCtrl &operator=(const SetpointCtrl &) = default;
  ~SetpointCtrl();

  void run();
};
#endif