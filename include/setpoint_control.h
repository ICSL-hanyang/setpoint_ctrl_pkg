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

class VehiclePos
{
private:
  ros::NodeHandle& rNH;
  const unsigned int id;
  tf2::Vector3 pos;
  tf2::Vector3 sum_sp;
  tf2::Vector3 err;
  tf2::Vector3 setpoint_pos;
  
  /* 현재 모드가 offboard 모드일때만 업데이트를 하기 위해 필요 */
  ros::Subscriber state_sub;
  mavros_msgs::State cur_state;

  void stateCB(const mavros_msgs::State::ConstPtr &msg);
public:
  VehiclePos() = delete;
  VehiclePos(const unsigned int&, ros::NodeHandle&);
  VehiclePos(const VehiclePos&);
  ~VehiclePos();

  const unsigned int getID() const;
  void setPos(const tf2::Vector3&);
  tf2::Vector3 getPos() const;
  void setSumOfSp(const tf2::Vector3&);
  tf2::Vector3 getSumOfSp() const;
  void setErr(const tf2::Vector3&);
  tf2::Vector3 getErr() const;
  void setSetpointPos(const tf2::Vector3&);
  tf2::Vector3 getSetpointPos() const;
  mavros_msgs::State getCurrntState() const;
};

class SetpointCtrl
{
private:
  ros::NodeHandle& rNH;
  ros::NodeHandle nh_global;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tfListener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

  std::vector<VehiclePos> vehicle_positions;

  static unsigned int num_drone;
  static double kp;
  static double kp_sp;
  static double range_sp;
  static double max_speed;

  void limit(tf2::Vector3&, double);
  void getVehiclePos(VehiclePos&);
  void separate(VehiclePos&);
  void seek(VehiclePos&);
  void transformSender(VehiclePos&);
  tf2::Vector3 cohesion(); /* 미구현 */


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