#ifndef UWV_H
#define UWV_H

#include <ros/ros.h>

// Message data-types
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include "darknet_ros_msgs/BoundingBoxes.h"

// Control libraries
#include "pid_translate.h"
#include "pid_rotate.h"

// Misc libraries
#include "stoppable_thread.h"
#include "config_UWV.h"
#include <cmath>
#define RAD2DEG 57.2957795131

// Total number of thrusters present in the UWV
#define THRUSTER_NUM 6

// Thruster location and index mapping
#define F_PORT 0 
#define F_STAR 1
#define M_PORT 2
#define M_STAR 3
#define B_PORT 4
#define B_STAR 5

// To store orientation details
struct Cardan
{
  double roll, pitch, yaw;
};

/**
 * @brief Acts as an abstract bridge between the control logic algorithms and the
 * hardware/software underwater vehicle (UWV). Any modifications to this class
 * would be reflected in both hardware and software UWV.
 */
class UWV : public StoppableThread
{
public:
  virtual ~UWV(){}
  
  // Message topics
  const std::string effort_topic_ = "/auv_v2/thruster_command";
  const std::string depth_topic_ = "/auv_v2/depth";
  const std::string imu_topic_ = "/auv_v2/imu";
  const std::string cam_topic_ = "/darknet_ros/bounding_boxes";

  // Effort publisher
  ros::Publisher effort_pub_;
  sensor_msgs::JointState effort_;

  // Depth data subscriber
  ros::Subscriber depth_subs_;

  // IMU data subscriber
  ros::Subscriber imu_subs_;
  
  // Camera data subscriber
  ros::Subscriber cam_subs_;

  // Control fusion constant {lies in [0,1]}
  float beta_;

  // Maximum effort for each thruster
  float max_thrust_;

  /**
   * @brief SYS(surge, yaw, sway) matrix.
   * A matrix containing multiplication factors for each thruster in the 
   * vectored configuration. This along with the fusion constant are used
   * to determine the effort for each of the thrusters.
   * Row #0: Surge factors 
   * Row #1: Yaw factors
   * Row #2: Sway factors
   * Column sequence: F_PORT, F_STAR, B_PORT, B_STAR
  */
  float sys_mat_[3][4] = {{1,1,1,1},{1,-1,1,-1},{1,-1,-1,1}};

  // PID controllers
  PidTranslate depth_controller_;
  PidRotate yaw_controller_;

  void pidConfig(void);
  virtual void heavePid2Effort(float pid_heave) = 0;
  virtual void vectoredPid2Effort(float pid_yaw, float pid_surge, float pid_sway) = 0;
  void depthCallbck(const geometry_msgs::PointStamped&);
  void imuCallbck(const sensor_msgs::Imu&);
  void drkntCallbck(const darknet_ros_msgs::BoundingBoxes& camData_);

  // Current x,y,z | x -> thrust; y -> thrust; z -> set-point
  float cur_xyz_[3];

  // Set x,y,z | x -> thrust; y -> thrust; z -> set-point
  float set_xyz_[3];

  // Current orientation
  Cardan cur_orient_;
  
  // Set orientation
  Cardan set_orient_;

  // To indicate whether UWV is traversing
  bool is_traversing_;

  // To store darknet output
  darknet_ros_msgs::BoundingBoxes detected_obj_;

  // PID related methods
  void run();
  void reconfigPid(bool yaw, bool depth);
  void showPidConfig(void);

  void initUWV(ros::NodeHandle& node, float max_th, float beta);
  void allStop(void);
  void setEffort(float* manual_effort);
  void sendCommands(void);
};

#endif  // UWV_H
