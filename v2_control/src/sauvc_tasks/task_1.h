#ifndef TASK_1
#define TASK_1

#define MOVE_LEFT 0
#define MOVE_RIGHT 1

short direction_flag = MOVE_LEFT;
darknet_ros_msgs::BoundingBoxes objs_;

namespace task_1
{
void detectObstacle()
{
  objs_ = my_auv_.detected_obj_;
  short size = objs_.bounding_boxes.size();

  ros::Duration(0.2).sleep();
  ros::spinOnce();

  objs_ = my_auv_.detected_obj_;
  size = objs_.bounding_boxes.size();

  // Looking for obstacle in the current field of view (if probability > 0.5 --> detected)
  for(int i = 0; i < size; i++)
  {
    if(objs_.bounding_boxes[i].Class.compare("obstacle") == 0)
    {
      if(objs_.bounding_boxes[i].probability >= 0.5)
      {
        obstacle_found_ = true;
        return;
      }
    }
  }
  obstacle_found_ = false;
}

void scanObstacle()
{
  if (direction_flag == MOVE_LEFT)
  {
    if (my_auv_.set_orient_.yaw >= -45)
    {
      move::doYaw(my_auv_.set_orient_.yaw - 6, 1);
    }
    else if (my_auv_.set_orient_.yaw < -45)
    {
      direction_flag = MOVE_RIGHT;
    }
  }

  if (direction_flag == MOVE_RIGHT)
  {
    if (my_auv_.set_orient_.yaw <= 45)
    {
      move::doYaw(my_auv_.set_orient_.yaw + 6, 1);
    }
    else
    {
      // Obstacle not found after performing yaw
      move::doYaw(0, 1);

      // Performing surge 
      move::doSurge(28, 6);
      direction_flag = MOVE_LEFT;
    }
  }
}

} // task_1

#endif
