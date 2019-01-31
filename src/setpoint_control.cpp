//연산자들 - http://docs.ros.org/latest/api/tf2/html/Vector3_8h_source.html#l00361
//http://docs.ros.org/latest/api/tf2/html/namespacetf2.html#a49192866d5c81a02cc25119c27d0a401
//http://wiki.ros.org/tf2

#include <ros/ros.h>
#include <setpoint_control.h>

/* VehiclePos */
VehiclePos::VehiclePos(const unsigned int &_id)
    : id(_id),
      pos(tf2::Vector3(0, 0, 0))
{
}

VehiclePos::VehiclePos(const VehiclePos &rhs)
    : id(rhs.id),
      pos(rhs.pos)
{
}

const unsigned int VehiclePos::getID() const
{
    return id;
}

void VehiclePos::setPos(const tf2::Vector3 &_r_pos)
{
    pos = _r_pos;
}

tf2::Vector3 VehiclePos::getPos() const
{
    return pos;
}

/* SetpointCtrl */
unsigned int SetpointCtrl::num_drone;
double SetpointCtrl::kp = 0.03;
double SetpointCtrl::kp_sp = 0.07;
double SetpointCtrl::range_sp = 3;
double SetpointCtrl::max_speed;

SetpointCtrl::SetpointCtrl(ros::NodeHandle &_rNH) : rNH(_rNH),
                                                    nh_global(ros::NodeHandle()),
                                                    tfBuffer(new tf2_ros::Buffer()),
                                                    tfListener(new tf2_ros::TransformListener(*tfBuffer)),
                                                    tf_br(new tf2_ros::TransformBroadcaster()),
                                                    position(tf2::Vector3(0, 0, 0))
{
    double _num_drone, _id;
    rNH.getParam("num_drone", _num_drone);
    rNH.getParam("id", _id);
    rNH.getParam("kp", kp);
    rNH.getParam("kp_sp", kp_sp);
    rNH.getParam("range_sp", range_sp);
    rNH.getParam("max_speed", max_speed);

    if (_num_drone < 1)
    {
        ROS_WARN("num_drone Param must be greater than 1");
        num_drone = 0;
    }
    if (_id < 1)
        ROS_WARN("id of drone Param must be greater than 1");

    num_drone = (unsigned int)_num_drone;
    my_id = (unsigned int)_id;

    state_sub = nh_global.subscribe("/camila" + std::to_string(my_id) + "/mavros/state", 10, &SetpointCtrl::stateCB, this);

    vehicle_positions.reserve(num_drone); // 드론 개수에 맞게 메모리를 할당하여 푸쉬백 할때마다 메모리를 재할당을 방지

    for (int i = 0; i < num_drone; i++)
    {
        vehicle_positions.push_back(VehiclePos(i + 1));
        // 기체 id는 1부터 여기 for 문만 id 신경 쓰기
    }
}

SetpointCtrl::~SetpointCtrl()
{
    state_sub.shutdown();
    tfBuffer.release();
    tfListener.release();
    tf_br.release();
}

void SetpointCtrl::limit(tf2::Vector3 &v, double _limit)
{
    if (v.length() > _limit)
    {
        v.normalize();
        v *= _limit;
    }
}

void SetpointCtrl::getNeighborPos()
{
    for (auto &pos : vehicle_positions)
    {

        geometry_msgs::TransformStamped transformStamped;
        tf2::Vector3 vehicle_pos(0, 0, 0);

        try
        {
            transformStamped = tfBuffer->lookupTransform("swarm_map",
                                                         "camila" + std::to_string(pos.getID()) + "_base_link",
                                                         ros::Time(0));
            vehicle_pos.setX(transformStamped.transform.translation.x);
            vehicle_pos.setY(transformStamped.transform.translation.y);
            vehicle_pos.setZ(transformStamped.transform.translation.z);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        if (pos.getID() != my_id)
            pos.setPos(vehicle_pos);
        else
            position = vehicle_pos;
    }
}

tf2::Vector3 SetpointCtrl::separate()
{
    tf2::Vector3 sum(0, 0, 0);
    unsigned int cnt = 0;
    for (auto &pos : vehicle_positions)
    {
        if (pos.getID() != my_id)
        {
            tf2::Vector3 diff = position - pos.getPos();
            float dist = diff.length();
            if (dist > 0 && dist < range_sp)
            {
                (diff.normalize()) /= dist;
                sum += diff;
                cnt++;
            }
        }
    }
    if (cnt > 0)
    {
        sum /= cnt;
        limit(sum, max_speed);
    }

    return sum;
}

tf2::Vector3 SetpointCtrl::seek()
{
    geometry_msgs::TransformStamped tf_stamped;
    tf2::Vector3 err(0, 0, 0);

    try
    {
        tf_stamped = tfBuffer->lookupTransform("camila" + std::to_string(my_id) + "_setpoint",
                                               "camila" + std::to_string(my_id) + "_target",
                                               ros::Time(0));
        err.setX(tf_stamped.transform.translation.x);
        err.setY(tf_stamped.transform.translation.y);
        err.setZ(tf_stamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    limit(err, max_speed);

    return err;
}

void SetpointCtrl::update()
{
    rNH.getParam("kp", kp);
    rNH.getParam("kp_sp", kp_sp);
    rNH.getParam("range_sp", range_sp);
    rNH.getParam("max_speed", max_speed);
    setpoint = position + (separate() * kp_sp + seek() * kp);
}

void SetpointCtrl::transformSender()
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "swarm_map";
    transformStamped.child_frame_id = "camila" + std::to_string(my_id) + "_setpoint";
    transformStamped.transform.translation.x = setpoint.getX();
    transformStamped.transform.translation.y = setpoint.getY();
    transformStamped.transform.translation.z = setpoint.getZ();

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_br->sendTransform(transformStamped);
}

void SetpointCtrl::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state = *msg;
}

tf2::Vector3 SetpointCtrl::cohesion()
{
}

void SetpointCtrl::run()
{
    if (cur_state.mode == "offboard" || cur_state.mode == "OFFBOARD")
    {
        getNeighborPos();
        update();
    }
    transformSender();
}