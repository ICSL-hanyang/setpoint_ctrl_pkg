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

    for (int i = 0; i < num_drone; i++)
    {
        vehicle_pos.push_back(tf2::Vector3(0, 0, 0));
    }
}

SwarmCtrl::~SwarmCtrl()
{
}

void SwarmCtrl::getNeighborPos()
{
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    int i = 1;
    for (iter = vehicle_pos.begin(); iter != vehicle_pos.end(); iter++)
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("camila" + std::to_string(id) + "_base_link",
                                                        "camila" + std::to_string(i) + "_base_link",
                                                        ros::Time(0) );
            //벡터로 바꿔서 전역 벡터 배열에 넣기
            // tf::Vector3 vect_1 (0,0,0);
            // vect_1 = transformStamped.getOrigin();
            // float dist_vect_1 = vect_1.distance(vect_1);
        }
        catch (tf2::TransformException &ex) //LookupException 인지 확인하기
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        i++;
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

tf2::Vector3 SwarmCtrl::seek(tf2::Vector3 _target)
{
}

tf2::Vector3 SwarmCtrl::separate(std::vector<tf2::Vector3> _vehicles)
{
    tf2::Vector3 sum(0, 0, 0);
}

tf2::Vector3 SwarmCtrl::cohesion(std::vector<tf2::Vector3> _vehicles)
{
}

void SwarmCtrl::update()
{
}
