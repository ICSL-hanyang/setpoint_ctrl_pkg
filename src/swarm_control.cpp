//연산자들 - http://docs.ros.org/latest/api/tf2/html/Vector3_8h_source.html#l00361
//http://docs.ros.org/latest/api/tf2/html/namespacetf2.html#a49192866d5c81a02cc25119c27d0a401
//http://wiki.ros.org/tf2

#include <ros/ros.h>
#include <swarm_control.h>

/* VehiclePos */
VehiclePos::VehiclePos(const unsigned int &_id)
    : id(_id),
      relative_pos(tf2::Vector3(0, 0, 0))
{
}

const unsigned int VehiclePos::getID()
{
    return id;
}

void VehiclePos::setRelativePos(const tf2::Vector3 &_r_pos)
{
    relative_pos = _r_pos;
}

tf2::Vector3 VehiclePos::getRelativePos()
{
    return relative_pos;
}

/* SwarmCtrl */
void SwarmCtrl::limit(tf2::Vector3 v, float _limit)
{
    if (tf2::tf2Distance(v, tf2::Vector3(0, 0, 0)) > _limit)
    {
        //ROS_INFO("limit %lf", v.distance(tf2::Vector3(0,0,0)));
        if (v.distance(tf2::Vector3(0, 0, 0)) != 0)
            v.normalize();
        v *= _limit; // 개별적으로 곱해지므로 크기가 limit 보다 커짐
    }
}

void SwarmCtrl::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

SwarmCtrl::SwarmCtrl(tf2::Vector3 _position) : nh(ros::NodeHandle("~")),
                                               nh_global(ros::NodeHandle()),
                                               position(_position),
                                               velocity(tf2::Vector3(0, 0, 0)),
                                               acceleration(tf2::Vector3(0, 0, 0)),
                                               m(1),
                                               k(1),
                                               b(1),
                                               range(2)
{
    float _num_drone, _id;
    nh.getParam("num_drone", _num_drone);
    nh.getParam("id", _id);
    nh.getParam("max_force", max_force);
    nh.getParam("max_speed", max_speed);
    nh.getParam("seek_weight", seek_weight);
    nh.getParam("separate_weight", separate_weight);

    if (_num_drone < 1)
    {
        ROS_WARN("num_drone Param must be greater than 1");
        num_drone = 0;
    }
    if (_id < 1)
        ROS_WARN("id of drone Param must be greater than 1");

    tfBuffer = new tf2_ros::Buffer();
    tfListener = new tf2_ros::TransformListener(*tfBuffer);

    num_drone = (unsigned int)_num_drone;
    my_id = (unsigned int)_id;

    state_sub = nh_global.subscribe("/camila" + std::to_string(my_id) + "/mavros/state", 10, &SwarmCtrl::stateCB, this);

    for (int i = 0; i < num_drone; i++)
    {
        vehicle_positions.push_back(VehiclePos(i + 1));
        // 기체 id는 1부터 여기 for 문만 id 신경 쓰기
    }
}

SwarmCtrl::~SwarmCtrl()
{
    state_sub.shutdown();
    delete tfBuffer;
    delete tfListener;
}

void SwarmCtrl::getNeighborPos()
{
    for (auto &pos : vehicle_positions)
    {
        if (pos.getID() != my_id)
        {
            geometry_msgs::TransformStamped transformStamped;
            tf2::Vector3 r_pos(0, 0, 0);

            try
            {
                transformStamped = tfBuffer->lookupTransform("camila" + std::to_string(pos.getID()) + "_base_link",
                                                             "camila" + std::to_string(my_id) + "_base_link",
                                                             ros::Time(0));
                r_pos.setX(transformStamped.transform.translation.x);
                r_pos.setY(transformStamped.transform.translation.y);
                r_pos.setZ(transformStamped.transform.translation.z);
                // printf("x = %f", vehicle_positions[i].getX());
                // printf("  y = %f", vehicle_positions[i].getY());
                // printf("  z = %f\n", vehicle_positions[i].getZ());
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            pos.setRelativePos(r_pos);
        }
    }
}

tf2::Vector3 SwarmCtrl::seek()
{
    geometry_msgs::TransformStamped tf_stamped;
    tf2::Vector3 desired(0, 0, 0);

    try
    {
        tf_stamped = tfBuffer->lookupTransform("camila" + std::to_string(my_id) + "_target",
                                               "camila" + std::to_string(my_id) + "_setpoint",
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

    float dist = desired.distance(tf2::Vector3(0, 0, 0));
    float damp_speed = dist * max_speed / range;

    ROS_INFO("Seek %lf", dist);
    if (dist != 0)
        desired.normalize();

    if (dist < range)
        desired *= damp_speed;
    else
        desired *= max_speed;

    /* tf2::Vector3 steer = desired - velocity;
    limit(steer, max_force);
    return steer; */
    return desired;
}

tf2::Vector3 SwarmCtrl::separate()
{
    tf2::Vector3 sum(0, 0, 0);
    unsigned int cnt = 0;
    for (auto &pos : vehicle_positions)
    {
        if (pos.getID() != my_id)
        {
            tf2::Vector3 diff = pos.getRelativePos();
            float dist = diff.distance(tf2::Vector3(0, 0, 0));
            //printf("dist = %f", dist);
            if (dist < range)
            {
                //ROS_INFO("Separate %lf", diff.distance(tf2::Vector3(0,0,0)));
                if (diff.distance(tf2::Vector3(0, 0, 0)) != 0)
                    diff.normalize();
                diff /= dist;
                sum += diff;
                cnt++;
            }
        }
    }
    if (cnt > 0)
    {
        sum /= cnt;
        //ROS_INFO("Sum %lf", sum.distance(tf2::Vector3(0,0,0)));
        if (sum.distance(tf2::Vector3(0, 0, 0)) != 0)
            sum.normalize();
        sum *= max_speed;
        sum -= velocity;
        limit(sum, max_force);
    }

    return sum;
}

tf2::Vector3 SwarmCtrl::cohesion()
{
}

void SwarmCtrl::applyForce(tf2::Vector3 _force)
{
    tf2::Vector3 f = _force;
    f /= m;
    acceleration += f;
}

void SwarmCtrl::applyBehaviors()
{
    tf2::Vector3 seekForce = seek() * seek_weight;
    tf2::Vector3 seperateForce = separate() * separate_weight;
    applyForce(seekForce);
    //applyForce(seperateForce);
}

void SwarmCtrl::update()
{
    velocity += acceleration;
    limit(velocity, max_speed);
    position += velocity;
    acceleration *= 0;
    // printf("x = %f", position.getX());
    // printf("   y = %f", position.getY());
    // printf("   z = %f\n", position.getZ());
}

void SwarmCtrl::transformSender()
{
    static tf2_ros::TransformBroadcaster tf_br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "swarm_map";
    transformStamped.child_frame_id = "camila" + std::to_string(my_id) + "_setpoint";
    transformStamped.transform.translation.x = position.getX();
    transformStamped.transform.translation.y = position.getY();
    transformStamped.transform.translation.z = position.getZ();

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_br.sendTransform(transformStamped);
}

void SwarmCtrl::run()
{
    if (cur_state.mode == "offboard" || cur_state.mode == "OFFBOARD")
    {
        getNeighborPos();
        applyBehaviors();
        update();
    }
    transformSender();
}