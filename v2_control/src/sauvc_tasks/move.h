#ifndef MOVE_FNS
#define MOVE_FNS

namespace move
{

/**
 * @brief Blocks the thread while executing ros::spinOnce()
 * @param time Time for which this delay should occur (in seconds)
 */ 
void spinningDelay(float time)
{
  unsigned long ticks = GET_COUNTDOWN_TICKS(time);
  ros::Rate rate(MOVE_SPIN_RATE);
  while(ticks != 1 && ros::ok())
  {
    ros::spinOnce();
    ticks--;
    rate.sleep();
  }
}

/**
 * @brief Performs yaw until the current position lies within the given sensitivity
 * @param angle Set-point to be achieved
 * @param sensitivity SET_POINT +- sensitivity -> defines the range in which if the bot it, it is considered as successful
 */ 
void doYaw(float angle, float sensitivity)
{
  my_auv_.set_orient_.yaw = angle;
  ROS_INFO_STREAM("Performing yaw");

  ros::Rate rate(MOVE_SPIN_RATE);
  while (fabs(my_auv_.set_orient_.yaw - my_auv_.cur_orient_.yaw) >= sensitivity && ros::ok())
  {
    ros::spinOnce();
    //rate.sleep();
  }
  ROS_INFO_STREAM("Yaw successful!");
}

/**
 * @brief Sets the given surge thrust for the given amount of time.
 * @param surge_thrust Amount of surge thrust
 * @param surge_time Time (in seconds) for which the thrust should be active
 *
 */
void doSurge(float surge_thrust, float surge_time)
{
  my_auv_.allStop();
  my_auv_.set_xyz_[0] = surge_thrust;
  ROS_INFO_STREAM("Performing surge");

  spinningDelay(surge_time);

  my_auv_.set_xyz_[0] = 0;
}

} // move

#endif
