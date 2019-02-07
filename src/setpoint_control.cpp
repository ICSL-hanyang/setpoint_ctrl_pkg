//연산자들 - http://docs.ros.org/latest/api/tf2/html/Vector3_8h_source.html#l00361
//http://docs.ros.org/latest/api/tf2/html/namespacetf2.html#a49192866d5c81a02cc25119c27d0a401
//http://wiki.ros.org/tf2

#include <ros/ros.h>
#include <setpoint_control.h>

/* VehiclePos */
VehiclePos::VehiclePos(ros::NodeHandle &rNH, const unsigned int &id)
    : rNH_(rNH),
      id_(id),
      pos_(tf2::Vector3(0, 0, 0)),
      sum_sp_(tf2::Vector3(0, 0, 0)),
      err_(tf2::Vector3(0, 0, 0)),
      setpoint_pos_(tf2::Vector3(0, 0, 0))
{
    state_sub_ = rNH_.subscribe("/camila" + std::to_string(id_) + "/mavros/state", 10, &VehiclePos::stateCB, this);
}

VehiclePos::VehiclePos(const VehiclePos &rhs)
    : rNH_(rhs.rNH_),
      id_(rhs.id_),
      pos_(rhs.pos_),
      sum_sp_(rhs.sum_sp_),
      err_(rhs.err_),
      setpoint_pos_(rhs.setpoint_pos_)
{
    state_sub_ = rNH_.subscribe("/camila" + std::to_string(id_) + "/mavros/state", 10, &VehiclePos::stateCB, this);
}

VehiclePos::~VehiclePos()
{
    state_sub_.shutdown();
}

void VehiclePos::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
    cur_state_ = *msg;
}

const unsigned int VehiclePos::getID() const
{
    return id_;
}

void VehiclePos::setPos(const tf2::Vector3 &pos)
{
    pos_ = pos;
}

tf2::Vector3 VehiclePos::getPos() const
{
    return pos_;
}

void VehiclePos::setSumOfSp(const tf2::Vector3 &sum_sp)
{
    sum_sp_ = sum_sp;
}

tf2::Vector3 VehiclePos::getSumOfSp() const
{
    return sum_sp_;
}

void VehiclePos::setErr(const tf2::Vector3 &err)
{
    err_ = err;
}

tf2::Vector3 VehiclePos::getErr() const
{
    return err_;
}

void VehiclePos::setSetpointPos(const tf2::Vector3 &setpoint_pos)
{
    setpoint_pos_ = pos_ + setpoint_pos;
}

tf2::Vector3 VehiclePos::getSetpointPos() const
{
    return setpoint_pos_;
}

mavros_msgs::State VehiclePos::getCurrntState() const
{
    return cur_state_;
}

/* SetpointCtrl */
unsigned int SetpointCtrl::num_drone_;
double SetpointCtrl::kp_ = 0.03;
double SetpointCtrl::kp_sp_ = 0.07;
double SetpointCtrl::range_sp_ = 3;
double SetpointCtrl::max_speed_;

SetpointCtrl::SetpointCtrl(ros::NodeHandle &rNH)
    : rNH_(rNH),
      nh_global_(""),
      tfBuffer_(new tf2_ros::Buffer()),
      tfListener_(new tf2_ros::TransformListener(*tfBuffer_)),
      tf_br_(new tf2_ros::TransformBroadcaster())
{
    double num_drone;
    rNH_.getParam("num_drone", num_drone);
    rNH_.getParam("kp", kp_);
    rNH_.getParam("kp_sp", kp_sp_);
    rNH_.getParam("range_sp", range_sp_);
    rNH_.getParam("max_speed", max_speed_);

    if (num_drone < 1)
    {
        ROS_WARN("num_drone Param must be greater than 1");
        num_drone_ = 0;
    }
    else
        num_drone_ = (unsigned int) num_drone;

    vehicle_positions_.reserve(num_drone_); // 드론 개수에 맞게 메모리를 할당하여 푸쉬백 할때마다 메모리를 재할당을 방지

    for (int i = 0; i < num_drone_; i++)
    {
        vehicle_positions_.push_back(VehiclePos(nh_global_, i + 1));
        // 기체 id는 1부터 여기 for 문만 id 신경 쓰기
    }
}

SetpointCtrl::~SetpointCtrl()
{
    tfBuffer_.release();
    tfListener_.release();
    tf_br_.release();
}

void SetpointCtrl::limit(tf2::Vector3 &v, const double &limit)
{
    if (v.length() > limit)
    {
        v.normalize();
        v *= limit;
    }
}

void SetpointCtrl::getVehiclePos()
{
    for (auto &pos : vehicle_positions_)
    {
        geometry_msgs::TransformStamped transformStamped;
        tf2::Vector3 vehicle_pos(0, 0, 0);

        try
        {
            transformStamped = tfBuffer_->lookupTransform("swarm_map",
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
        pos.setPos(vehicle_pos);
    }
}

void SetpointCtrl::separate(VehiclePos &pos)
{
    tf2::Vector3 sum(0, 0, 0);
    unsigned int cnt = 0;
    for (auto &another_pos : vehicle_positions_)
    {
        if (&pos != &another_pos)
        {
            tf2::Vector3 diff = pos.getPos() - another_pos.getPos();
            float dist = diff.length();
            if (dist > 0 && dist < range_sp_)
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
        limit(sum, max_speed_);
        pos.setSumOfSp(sum);
    }
}

void SetpointCtrl::seek(VehiclePos &pos)
{
    geometry_msgs::TransformStamped tf_stamped;
    tf2::Vector3 err_(0, 0, 0);
    unsigned int id = pos.getID();
    try
    {
        tf_stamped = tfBuffer_->lookupTransform("camila" + std::to_string(id) + "_setpoint",
                                               "camila" + std::to_string(id) + "_target",
                                               ros::Time(0));
        err_.setX(tf_stamped.transform.translation.x);
        err_.setY(tf_stamped.transform.translation.y);
        err_.setZ(tf_stamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    limit(err_, max_speed_);
    pos.setErr(err_);
}

void SetpointCtrl::transformSender(const VehiclePos &pos)
{
    tf2::Vector3 setpoint = pos.getSetpointPos();
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "swarm_map";
    transformStamped.child_frame_id = "camila" + std::to_string(pos.getID()) + "_setpoint";
    transformStamped.transform.translation.x = setpoint.getX();
    transformStamped.transform.translation.y = setpoint.getY();
    transformStamped.transform.translation.z = setpoint.getZ();

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    tf_br_->sendTransform(transformStamped);
}

tf2::Vector3 SetpointCtrl::cohesion()
{
}

void SetpointCtrl::run()
{
    rNH_.getParam("kp", kp_);
    rNH_.getParam("kp_sp", kp_sp_);
    rNH_.getParam("range_sp", range_sp_);
    rNH_.getParam("max_speed", max_speed_);
    getVehiclePos();
    for (auto &pos : vehicle_positions_)
    {
        std::string mode(pos.getCurrntState().mode);
        if (mode == "OFFBOARD" || mode == "offboard")
        {
            separate(pos);
            seek(pos);

            pos.setSetpointPos(pos.getSumOfSp() * kp_sp_ + pos.getErr() * kp_);
        }
        transformSender(pos);
    }
}