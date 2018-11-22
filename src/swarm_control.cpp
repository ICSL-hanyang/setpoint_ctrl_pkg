//연산자들 - http://docs.ros.org/latest/api/tf2/html/Vector3_8h_source.html#l00361
//http://docs.ros.org/latest/api/tf2/html/namespacetf2.html#a49192866d5c81a02cc25119c27d0a401
//http://wiki.ros.org/tf2

#include <ros/ros.h>
#include <swarm_control.h>

SwarmCtrl::SwarmCtrl(tf2::Vector3 _position) : nh(ros::NodeHandle("~")),
                                               position(_position),
                                               velocity(tf2::Vector3(0, 0, 0)),
                                               acceleration(tf2::Vector3(0, 0, 0)),
                                               m(1)
{
    float _num_drone, _id;
    nh.getParam("num_drone", _num_drone);
    nh.getParam("id", _id);
    nh.getParam("max_force", max_force);
    nh.getParam("max_speed", max_speed);

    num_drone = (unsigned int)_num_drone;
    id = (unsigned int)_id;

    for (int i = 0; i < num_drone + 1; i++)
    {
        vehicle_pos.push_back(tf2::Vector3(0, 0, 0));
    }
}

SwarmCtrl::~SwarmCtrl()
{
}

void SwarmCtrl::limit(tf2::Vector3 v, float _limit)
{
    if (v.getX() > _limit)
        v.setX(_limit);
    if (v.getY() > _limit)
        v.setY(_limit);
    if (v.getZ() > _limit)
        v.setZ(_limit);
}

void SwarmCtrl::getNeighborPos()
{
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    for (int i = 1; i < vehicle_pos.size(); i++)
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("camila" + std::to_string(id) + "_base_link",
                                                        "camila" + std::to_string(i) + "_base_link",
                                                        ros::Time(0));
            vehicle_pos[i].setX(transformStamped.transform.translation.x);
            vehicle_pos[i].setY(transformStamped.transform.translation.y);
            vehicle_pos[i].setZ(transformStamped.transform.translation.z);
        }
        catch (tf2::TransformException &ex) //LookupException 인지 확인하기
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
}

void SwarmCtrl::applyForce(tf2::Vector3 _force)
{
    tf2::Vector3 f = _force;
    f /= m;
    acceleration += f;
}

void SwarmCtrl::applyBehaviors(std::vector<tf2::Vector3> _vehicles)
{
}

tf2::Vector3 SwarmCtrl::seek()
{
    geometry_msgs::TransformStamped tf_stamped;
    tf2::Vector3 desired(0, 0, 0);
    try
    {
        tf_stamped = tfBuffer.lookupTransform("camila" + std::to_string(id) + "_base_link",
                                              "camila" + std::to_string(id) + "_target",
                                              ros::Time(0));
        desired.setX(tf_stamped.transform.translation.x);                 
        desired.setY(tf_stamped.transform.translation.y);                 
        desired.setZ(tf_stamped.transform.translation.z);                 
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    float dist = tf2::tf2Distance(desired, tf2::Vector3(0,0,0));
    float damp_speed = dist * max_speed / range;
    desired.normalize();

    if( dist < range)
        desired *= damp_speed;
    else
        desired *= max_speed;

    tf2::Vector3 steer = desired -velocity;
    limit(steer, max_force);
    return steer;
}

tf2::Vector3 SwarmCtrl::separate()
{
    tf2::Vector3 sum(0, 0, 0);
    unsigned int cnt = 0;
    for (iter = vehicle_pos.begin(); iter != vehicle_pos.end(); iter++)
    {
        float dist = tf2::tf2Distance(vehicle_pos[id], *iter);
        if (dist < range)
        {
            tf2::Vector3 diff = vehicle_pos[id] - *iter;
            diff.normalize();
            diff /= dist;
            sum += diff;
            cnt++;
        }
    }
    if (cnt > 0)
    {
        sum /= cnt;
        sum.normalize();
        sum *= max_speed;
        sum -= velocity;
        limit(sum, max_force);
    }

    return sum;
}

tf2::Vector3 SwarmCtrl::cohesion(std::vector<tf2::Vector3> _vehicles)
{
}

void SwarmCtrl::update()
{
    getNeighborPos();
    velocity += acceleration;
    limit(velocity, max_speed);
    position += velocity;
    acceleration *= 0;
}
