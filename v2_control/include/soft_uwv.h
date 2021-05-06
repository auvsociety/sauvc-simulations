#ifndef SOFT_UWV_H
#define SOFT_UWV_H

#include "uwv.h"

/**
 * @brief Child class of UWV defining the how PID values and the thrust is mapped
 * for soft UWV.
 */
class SoftUWV : public UWV
{
public:

  SoftUWV(){}

  virtual ~SoftUWV(){}

  /**
   * @brief Converts the output of PID into effort for motion along Z direction
   * (heave)
   * @param pid_heave Output of PID
   */
  void heavePid2Effort(float pid_heave) override
  {
    if(pid_heave > max_thrust_ - 8){
      pid_heave = max_thrust_ - 8; // limiting thrust
    }
    effort_.effort[M_PORT] = pid_heave;
    effort_.effort[M_STAR] = pid_heave;
  }

  /**
   * @brief Converts the output of PIDs into effort for rotation along Z axis
   * (yaw), surge, and sway.
   * @param pid_surge Ouput of surge PID/manual surge effort
   * @param pid_yaw Output of yaw PID
   * @param pid_sway Output of sway PID/manual sway effort
   */
  void vectoredPid2Effort(float pid_surge, float pid_yaw, float pid_sway) override
  {
    /**
     * effort = ((1-beta)/2)*surge_factor*pid_surge + beta*yaw_factor_pid_yaw
     *                                              + ((1-beta)/2)*sway_factor*pid_sway
     */ 

    effort_.effort[F_PORT] = ((1-beta_)/2)*sys_mat_[0][0]*pid_surge + beta_*sys_mat_[1][0]*pid_yaw + ((1-beta_)/2)*sys_mat_[2][0]*pid_sway;
    effort_.effort[F_STAR] = ((1-beta_)/2)*sys_mat_[0][1]*pid_surge + beta_*sys_mat_[1][1]*pid_yaw + ((1-beta_)/2)*sys_mat_[2][1]*pid_sway;
    effort_.effort[B_PORT] = ((1-beta_)/2)*sys_mat_[0][2]*pid_surge + beta_*sys_mat_[1][2]*pid_yaw + ((1-beta_)/2)*sys_mat_[2][2]*pid_sway;
    effort_.effort[B_STAR] = ((1-beta_)/2)*sys_mat_[0][3]*pid_surge + beta_*sys_mat_[1][3]*pid_yaw + ((1-beta_)/2)*sys_mat_[2][3]*pid_sway;
    
    // limiting thrust
    if(effort_.effort[F_PORT] > max_thrust_) effort_.effort[F_PORT] = max_thrust_ - 8; 
    if(effort_.effort[F_STAR] > max_thrust_) effort_.effort[F_STAR] = max_thrust_ - 8;
    if(effort_.effort[B_PORT] > max_thrust_) effort_.effort[B_PORT] = max_thrust_ - 8;
    if(effort_.effort[B_STAR] > max_thrust_) effort_.effort[B_STAR] = max_thrust_ - 8;

  }

};

#endif