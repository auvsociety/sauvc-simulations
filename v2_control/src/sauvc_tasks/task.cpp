#include<task.h>
#include<move.h>


/**
 * @brief Function to detect a particular object in the current camera frame.
 * @param objDetect A string representing the obstacle to be detected.
 * @returns A boolean confirming if the given obstacle was detected or not.
 */
bool task::detectObstacle(std::string objDetect)
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
        if(objs_.bounding_boxes[i].Class.compare(objDetect) == 0)
        {
            if(objs_.bounding_boxes[i].probability >= 0.5)
            {
                return true;
            }
        }
    }
    return false;
} 

/**
 * @brief A example class inherited from task class. 
 */
class task1: public task
{
    public:
        void run(){
            move::doSurge(10, 5);
        }
}