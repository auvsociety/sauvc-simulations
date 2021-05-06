#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "sensor_math.h"

// If set displays quaternion read by IMU
#define SHOW_QUAT 0

Quaternion q;
Cardan angles;
double depth;

/**
 * @brief Callback function listening to depth sensor's output.
 */
void DepthCallbck(const geometry_msgs::PointStamped& height_)
{
  depth = -height_.point.z;
}

void ImuCallbck(const sensor_msgs::Imu& imu_)
{
  q.w = imu_.orientation.w;
  q.x = imu_.orientation.x;
  q.y = imu_.orientation.y;
  q.z = imu_.orientation.z;

  angles = getCardanAngles(q);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loc_sensors");
  ros::NodeHandle node;

  ros::Subscriber depth_subs_ = node.subscribe("/auv_v2/depth", 500, DepthCallbck);
  ros::Subscriber imu_subs_ = node.subscribe("/auv_v2/imu", 500, ImuCallbck);

  if (depth_subs_.getTopic() != "")
    ROS_INFO("found depth sensor topic");
  else
    ROS_WARN("cannot find depth sensor topic!");
  if (imu_subs_.getTopic() != "")
    ROS_INFO("found imu topic");
  else
    ROS_WARN("cannot find imu topic!");

  ros::Rate rate(20);

  while (ros::ok())
  {
    std::cout << "Time (in secs): " << ros::Time::now().toSec() << std::endl
              << "Depth (in m): " << depth << std::endl
              << "Yaw (deg): " << -angles.yaw << std::endl
              << "Pitch (deg): " << angles.pitch << std::endl
              << "Roll (deg): " << angles.roll << std::endl;
#if SHOW_QUAT
    std::cout << "qx: " << q.x << std::endl
              << "qy: " << q.y << std::endl
              << "qz: " << q.z << std::endl
              << "qw: " << q.w << std::endl;
#endif
    std::cout << "---------" << std::endl << std::endl;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
