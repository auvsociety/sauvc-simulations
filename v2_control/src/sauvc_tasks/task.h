#ifndef TASK
#define TASK

#include<auv.h>

short direction_flag = MOVE_LEFT;
darknet_ros_msgs::BoundingBoxes objs_;

/**
 * @brief An abstract base class for implementing different tasks and mission planning for achieving autonomy 
 * of the vehicle. This class can be inherited easily and task-dependent functions can be added further.
 * Make sure to override the run() function.
 */
class task
{
  public:
    int taskID;
    bool isTaskRunning;


    // Parameterized constructor
    task(int taskID)
    {
      this->taskID = taskID;
      this->isTaskRunning = isTaskRunning;
    }
    
    bool detectObstacle();

    // Pure virtual function to execute the task
    virtual void run() = 0;
} 


#endif
