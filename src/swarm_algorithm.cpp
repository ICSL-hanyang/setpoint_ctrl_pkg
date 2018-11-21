//연산자들 - http://docs.ros.org/latest/api/tf2/html/Vector3_8h_source.html#l00361
//http://docs.ros.org/latest/api/tf2/html/namespacetf2.html#a49192866d5c81a02cc25119c27d0a401
//http://wiki.ros.org/tf2

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/Vector3.h>

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
void getNeighbors() //
{
    geometry_msgs::TransformStamped transformStamped;

    for (size_t i = 0; i < count; i++) //전체 몇대 인지를 어떻게 받을것이가 param?
    {
        try
        {
            transformStamped = tfBuffer.lookupTransform("camila@_base_link", "camila@_base_link",
                                                        ros::Time(0));
            //벡터로 바꿔서 전역 벡터 배열에 넣기
            tf::Vector3 vect_1 (0,0,0);
            vect_1 = transformStamped.getOrigin();
            float dist_vect_1 = vect_1.distance(vect_1);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
}

Vector3 seek() {}
Vector3 seperate() {}
