#ifndef HARD_UWV_H
#define HARD_UWV_H

#include "uwv.h"

/**
 * @brief Child class of UWV defining the how PID values and the thrust is mapped
 * for hard UWV.
 */
class HardUWV : public UWV
{
public:

  HardUWV(){}

  virtual ~HardUWV(){}

  /**
   * @brief Converts the output of PID into effort for motion along Z direction
   * (heave)
   * @param pid_heave Output of PID
   */
  void heavePid2Effort(float pid_heave) override
  {
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
  }

};

#endif