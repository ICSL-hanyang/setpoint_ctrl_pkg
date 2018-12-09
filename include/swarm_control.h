#ifndef SWARM_CONTROL_H
#define SWARM_CONTROL_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class SwarmCtrl
{
  private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;

    unsigned int num_drone;
    unsigned int id;

    std::vector<tf2::Vector3> vehicle_pos;
    std::vector<tf2::Vector3>::iterator iter;

    tf2::Vector3 position;
    tf2::Vector3 velocity;
    tf2::Vector3 acceleration;

    float m;
    float k;
    float b;
    float range;
                                                                                                                                                                 
    float max_force;
    float max_speed;
    float seek_weight;
    float separate_weight;
    void limit(tf2::Vector3, float);

  public:
    SwarmCtrl(tf2::Vector3);
    SwarmCtrl(SwarmCtrl &&) = default;
    SwarmCtrl(const SwarmCtrl &) = default;
    SwarmCtrl &operator=(SwarmCtrl &&) = default;
    SwarmCtrl &operator=(const SwarmCtrl &) = default;
    ~SwarmCtrl();

    void getNeighborPos();
    void applyForce(tf2::Vector3);
    void applyBehaviors();
    tf2::Vector3 seek();
    tf2::Vector3 separate();
    tf2::Vector3 cohesion(std::vector<tf2::Vector3>);
    void update();
    void transformSender();
    void run();
};

#endif