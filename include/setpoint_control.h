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
  tf2::Vector3 pos;

public:
  VehiclePos() = delete;
  VehiclePos(const unsigned int&);
  VehiclePos(const VehiclePos&);

  const unsigned int getID() const;
  void setPos(const tf2::Vector3&);
  tf2::Vector3 getPos() const;
};

class SetpointCtrl
{
private:
  ros::NodeHandle& rNH;
  ros::NodeHandle nh_global;
  
  /* 현재 모드가 offboard 모드일때만 업데이트를 하기 위해 필요 */
  ros::Subscriber state_sub;
  mavros_msgs::State cur_state;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

  std::vector<VehiclePos> vehicle_positions;

  tf2::Vector3 position;
  tf2::Vector3 setpoint;

  static unsigned int num_drone;
  static double kp;
  static double kp_sp;
  static double range_sp;
  static double max_speed;

  void limit(tf2::Vector3&, double);
  void getNeighborPos();
  tf2::Vector3 separate();
  tf2::Vector3 seek();
  void update();
  void transformSender();
  tf2::Vector3 cohesion(); /* 미구현 */

  void stateCB(const mavros_msgs::State::ConstPtr &msg);

public:
  SetpointCtrl(ros::NodeHandle&);
  SetpointCtrl(SetpointCtrl &&) = default;
  SetpointCtrl(const SetpointCtrl &) = default;
  SetpointCtrl &operator=(SetpointCtrl &&) = default;
  SetpointCtrl &operator=(const SetpointCtrl &) = default;
  ~SetpointCtrl();

  void run();
};
#endif