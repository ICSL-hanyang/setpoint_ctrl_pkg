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
  ros::NodeHandle& rNH_;
  const unsigned int id_;
  tf2::Vector3 pos_;
  tf2::Vector3 sum_sp_;
  tf2::Vector3 err_;
  tf2::Vector3 setpoint_pos_;
  
  /* 현재 모드가 offboard 모드일때만 업데이트를 하기 위해 필요 */
  ros::Subscriber state_sub_;
  mavros_msgs::State cur_state_;

  void stateCB(const mavros_msgs::State::ConstPtr &);
public:
  VehiclePos() = delete;
  VehiclePos(ros::NodeHandle&, const unsigned int&);
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
  ros::NodeHandle& rNH_;
  ros::NodeHandle nh_global_;

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  std::vector<VehiclePos> vehicle_positions_;

  static unsigned int num_drone_;
  static double kp_;
  static double kp_sp_;
  static double range_sp_;
  static double max_speed_;

  void limit(tf2::Vector3&,const double &);
  void getVehiclePos();
  void separate(VehiclePos&);
  void seek(VehiclePos&);
  void transformSender(const VehiclePos&);
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