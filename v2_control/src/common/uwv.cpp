#include "uwv.h"

/**
 * @brief Initializes the communication with the underwater vehicle by creating
 * publishers and subscribers.
 * @param node Reference to the node handle of the UWV's controller
 * @param max_th maximum effort for the thrusters
 * @param beta constant for vectored-thruster fusion (Read /simulations/docs to learn more about this)
 */
void UWV::initUWV(ros::NodeHandle& node, float max_th, float beta)
{
  // Initialize publisher
  effort_pub_ = node.advertise<sensor_msgs::JointState>(effort_topic_, 1024);

  // Initialize and check for subscription
  depth_subs_ = node.subscribe(depth_topic_, 1, &UWV::depthCallbck, this);
  if (depth_subs_)
  {
    ROS_INFO_STREAM("Subscribed to topic: " << depth_topic_);
  }
  else
  {
    ROS_WARN_STREAM("Could not subscribe to topic: " << depth_topic_);
  }

  imu_subs_ = node.subscribe(imu_topic_, 1, &UWV::imuCallbck, this);
  if (imu_subs_)
  {
    ROS_INFO_STREAM("Subscribed to topic: " << imu_topic_);
  }
  else
  {
    ROS_WARN_STREAM("Could not subscribe to topic: " << imu_topic_);
  }

  cam_subs_ = node.subscribe(cam_topic_, 1, &UWV::drkntCallbck, this);
  if (cam_subs_)
  {
    ROS_INFO_STREAM("Subscribed to topic: " << cam_topic_);
  }
  else
  {
    ROS_WARN_STREAM("Could not subscribe to topic: " << cam_topic_);
  }

  // initialize controllers
  ROS_INFO_STREAM("Configuring PID controllers...");
  depth_controller_ = PidTranslate();
  yaw_controller_ = PidRotate();
  pidConfig();
  ROS_INFO_STREAM("PID configured.");

  // Setting default values
  effort_.name.resize(THRUSTER_NUM);
  effort_.effort.resize(THRUSTER_NUM);
  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    effort_.effort[i] = 0.0;
  }
  effort_.name[F_PORT] = "f_port";
  effort_.name[F_STAR] = "f_star";
  effort_.name[M_PORT] = "m_port";
  effort_.name[M_STAR] = "m_star";
  effort_.name[B_PORT] = "b_port";
  effort_.name[B_STAR] = "b_star";
  for(int i = 0; i < 3; i++){
    set_xyz_[i] = 0;
    cur_xyz_[i] = 0;
  }
  set_orient_.yaw = 0;
  set_orient_.pitch = 0;
  set_orient_.roll = 0;
  cur_orient_.yaw = 0;
  cur_orient_.pitch = 0;
  cur_orient_.roll = 0;

  is_traversing_ = false;
  max_thrust_ = max_th;
  beta_ = beta;
  ROS_INFO_STREAM("UWV ready!!!");
}

/**
 * @brief Sets the PID gain values from the ROS parameter server. Default values
 * are used if parameters are not set.
 * Default: p: 10 | i: 0.1 | d: 0.5 | snstvty_: 0.01
 */
void UWV::pidConfig(void)
{
  ROS_INFO_STREAM("Configuring PID parameters...");

  // Depth
  if (ros::param::has(DEPTH_P))
  {
    ros::param::get(DEPTH_P, depth_controller_.kp_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_P << " Using default value: 10");
    depth_controller_.kp_ = 10;
  }

  if (ros::param::has(DEPTH_I))
  {
    ros::param::get(DEPTH_I, depth_controller_.ki_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_I << " Using default value: 0.1");
    depth_controller_.ki_ = 0.1;
  }

  if (ros::param::has(DEPTH_D))
  {
    ros::param::get(DEPTH_D, depth_controller_.kd_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_D << " Using default value: 0.5");
    depth_controller_.kd_ = 0.5;
  }

  if (ros::param::has(DEPTH_SNSTVTY))
  {
    ros::param::get(DEPTH_SNSTVTY, depth_controller_.snstvty_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << DEPTH_SNSTVTY << " Using default value: 0.01");
    depth_controller_.snstvty_ = 0.01;
  }

  // Yaw
  if (ros::param::has(YAW_P))
  {
    ros::param::get(YAW_P, yaw_controller_.kp_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_P << " Using default value: 10");
    yaw_controller_.kp_ = 10;
  }

  if (ros::param::has(YAW_I))
  {
    ros::param::get(YAW_I, yaw_controller_.ki_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_I << " Using default value: 0.1");
    yaw_controller_.ki_ = 0.1;
  }

  if (ros::param::has(YAW_D))
  {
    ros::param::get(YAW_D, yaw_controller_.kd_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_D << " Using default value: 0.5");
    yaw_controller_.kd_ = 0.5;
  }

  if (ros::param::has(YAW_SNSTVTY))
  {
    ros::param::get(YAW_SNSTVTY, yaw_controller_.snstvty_);
  }
  else
  {
    ROS_WARN_STREAM("Did not find parameter: " << YAW_SNSTVTY << " Using default value: 0.01");
    yaw_controller_.snstvty_ = 0.01;
  }

  ROS_INFO_STREAM("Loaded all PID parameters.");
}

/**
 * @brief Resets PID controllers of the parameters
 * @param yaw resets yaw PID controller if true
 * @param depth resets yaw PID controller if true
 */
void UWV::reconfigPid(bool yaw, bool depth){
  if(yaw)   yaw_controller_.reset();
  if(depth) depth_controller_.reset();
  pidConfig();
}

/**
 * @brief Displays current configuration of PID gains of all the controllers
 */ 
void UWV::showPidConfig(void){
  ROS_INFO_STREAM("\nYaw P gain: " << yaw_controller_.kp_ << std::endl
                  << "Yaw I gain: " << yaw_controller_.ki_ << std::endl
                  << "Yaw D gain: " << yaw_controller_.kd_ << std::endl
                  << "Yaw sensitivity: " << yaw_controller_.snstvty_ << std::endl);
  ROS_INFO_STREAM("\nDepth P gain: " << depth_controller_.kp_ << std::endl
                  << "Depth I gain: " << depth_controller_.ki_ << std::endl
                  << "Depth D gain: " << depth_controller_.kd_ << std::endl
                  << "Depth sensitivity: " << depth_controller_.snstvty_ << std::endl);
}

/**
 * @brief Runs the PID loop in the background until UWV_obj.stop() is called.
 */ 
void UWV::run(){
  // variables for real-time operation
  double prev_time = ros::Time::now().toSec();
  double cur_time;
  double dt;

  ROS_INFO_STREAM("Starting PID thread in background...");

  // Check if thread is requested to stop ?
  while (stopRequested() == false)
  {
    cur_time = ros::Time::now().toSec();

    dt = cur_time - prev_time;

    // sanity check
    if (dt <= 0)
      continue;
    heavePid2Effort(depth_controller_.update(set_xyz_[2], cur_xyz_[2], dt));
    vectoredPid2Effort(set_xyz_[0],
                       yaw_controller_.update(set_orient_.yaw, cur_orient_.yaw, dt),
                       set_xyz_[1]);
    sendCommands();

    prev_time = cur_time;
  }
  ROS_INFO_STREAM("PID thread killed.");
}

/**
 * @brief Stops all the thrusters.
 */
void UWV::allStop(void)
{
  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    effort_.effort[i] = 0;
  }
  effort_pub_.publish(effort_);
}

/**
 * @brief Manually set the effort values to the thrusters.
 * @param manual_effort Pointer to the first element of the array containing the
 * effort values for each thruster.
 */
void UWV::setEffort(float* manual_effort)
{
  if (!is_traversing_)
  {
    ROS_INFO_STREAM("UWV is not in the traversing mode.");
    return;
  }

  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    effort_.effort[i] = *(manual_effort + i);
  }
  effort_pub_.publish(effort_);
}

/**
 * @brief Publishes the commands
 */ 
void UWV::sendCommands(void)
{
  effort_pub_.publish(effort_);
}

/**
 * @brief Callback function listening to depth sensor's output.
 */
void UWV::depthCallbck(const geometry_msgs::PointStamped& depth_)
{
  cur_xyz_[2] = -depth_.point.z; // negating to conform to accepted convention
}

/**
 * @brief Callback function listening to IMU's output.
 */
void UWV::imuCallbck(const sensor_msgs::Imu& imu_)
{
  double sqw = imu_.orientation.w * imu_.orientation.w;
  double sqx = imu_.orientation.x * imu_.orientation.x;
  double sqy = imu_.orientation.y * imu_.orientation.y;
  double sqz = imu_.orientation.z * imu_.orientation.z;

  cur_orient_.roll = atan2(2.0 * (imu_.orientation.y * imu_.orientation.z + imu_.orientation.x * imu_.orientation.w),
                          (-sqx - sqy + sqz + sqw)) *
                    RAD2DEG;
  cur_orient_.pitch = asin(2.0 * (imu_.orientation.y * imu_.orientation.w - imu_.orientation.x * imu_.orientation.z) /
                          (sqx + sqy + sqz + sqw)) *
                     RAD2DEG;
  cur_orient_.yaw = -atan2(2.0 * (imu_.orientation.x * imu_.orientation.y + imu_.orientation.z * imu_.orientation.w),
                         (sqx - sqy - sqz + sqw)) *
                   RAD2DEG; // negating to conform to accepted convention
}


/**
 * @brief Callback function listening to darknet's output.
 */
void UWV::drkntCallbck(const darknet_ros_msgs::BoundingBoxes& camData_)
{
  detected_obj_ = camData_;
}
