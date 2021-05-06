/**
 * This is the main file which will be executed for autonomous movement of the vehicle.
 * Update #1 | Jan 20, 2021 | Autonomous detection of gate added
 * Update #2 | Feb 11, 2021 | Re-wrote the autonomy level 1 to suit new modular UWV
 */

#include "auv.h"

// Move-Headerfile (Contains all the functions to move AUV inside the pool)
#include "move.h"

// task-1 headerfiule (Contains all the functions required for performing task-1)
#include "task_1.h"

/***************** ROV MODE ********************/

/**
 * @brief Reads user's command and executes the corresponding function.
 */
void readInput(void)
{
  char input;

  while (do_not_quit_)
  {
    input = getch();

    switch (input)
    {
      case 'X':
        do_not_quit_ = false;
        break;
      default:
      case 'Z':
        displayHelp();
        break;

      // Start/stop traversing
      case 't':
        if (my_auv_.is_traversing_)
        {
          my_auv_.is_traversing_ = false;
          mode_ = IDLE_STATE;
          ROS_INFO_STREAM("Exiting traversing mode.");
        }
        else
        {
          my_auv_.is_traversing_ = true;
          my_auv_.set_xyz_[2] = 0.7;
          mode_ = IDLE_STATE;
          ROS_INFO_STREAM("Entering traversing & idle state...\n Depth: 0.7m");
        }
        break;
      
      // Toggle ROV mode
      case 'r':
        if (mode_ == ROV_MODE)
        {
          mode_ = IDLE_STATE;
          ROS_INFO_STREAM("Exiting ROV mode. Entered idle state.");
        }
        else
        {
          mode_ = ROV_MODE;
          ROS_INFO_STREAM("Entering ROV mode...");
        }
        break;
      // Toggle AUV mode
      case 'R':
        if (mode_ == AUV_MODE)
        {
          mode_ = IDLE_STATE;
          ROS_INFO_STREAM("Exiting AUV mode. Entered idle state.");
        }
        else
        {
          mode_ = AUV_MODE;
          ROS_INFO_STREAM("Entering AUV mode...");
        }
        break;

      // Refresh PID configuration
      case 'P':
        my_auv_.is_traversing_ = false;
        ROS_INFO_STREAM("Exiting traversing mode.");
        my_auv_.reconfigPid(true, true);
        ROS_INFO_STREAM("PID configuration reloaded. Entering traversing mode.");
        my_auv_.is_traversing_ = true;
        break;
      // Display all PID parameters
      case 'p': my_auv_.showPidConfig();
                break;
      
      case 'v': showVehicleStatus();
                break;

      // Stop thrusters
      case 'S':
        my_auv_.allStop();
        my_auv_.set_xyz_[0] = my_auv_.set_xyz_[1] = 0;
        ROS_INFO_STREAM("Sending thruster command: ZERO to all thrusters.");
        break;

      // Heave
      case 'W':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[2] += HEAVE_PLUS;
        ROS_INFO_STREAM("Set depth: " << my_auv_.set_xyz_[2]);
        break;
      case 'Q':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[2] -= HEAVE_MINUS;
        if (my_auv_.set_xyz_[2] < 0)
          my_auv_.set_xyz_[2] = 0;         // cannot give set point that is above water level
        ROS_INFO_STREAM("Set depth: " << my_auv_.set_xyz_[2]);
        break;

      // Yaw
      case 'd':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_orient_.yaw += YAW_PLUS;
        if(my_auv_.set_orient_.yaw >= 180.01){
					my_auv_.set_orient_.yaw = -(360 - my_auv_.set_orient_.yaw);
				}
        ROS_INFO_STREAM("Set heading: " << my_auv_.set_orient_.yaw);
        break;
      case 'a':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_orient_.yaw -= YAW_MINUS;
        if(my_auv_.set_orient_.yaw <= -179.9){
					my_auv_.set_orient_.yaw = 180;
				}
        ROS_INFO_STREAM("Set heading: " << my_auv_.set_orient_.yaw);
        break;
      
      // Surge
      case 'w':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[0] += SURGE_PLUS;
        ROS_INFO_STREAM("Surging effort: " << my_auv_.set_xyz_[0]);
        break;
      case 's':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[0] -= SURGE_MINUS;
        ROS_INFO_STREAM("Surging effort: " << my_auv_.set_xyz_[0]);
        break;

      // Sway
      case 'e':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[1] += SWAY_PLUS;
        ROS_INFO_STREAM("Sway effort: " << my_auv_.set_xyz_[1]);
        break;
      case 'q':
        if(!my_auv_.is_traversing_)
        {
          ROS_INFO_STREAM("Not in traversing mode.");
          break;
        }
        if(mode_ != ROV_MODE ){
          ROS_INFO_STREAM("Not in ROV mode.");
          break;
        }
        my_auv_.set_xyz_[1] -= SWAY_MINUS;
        ROS_INFO_STREAM("Sway effort: " << my_auv_.set_xyz_[1]);
        break;
    }
  }
}

/**
 * @brief Shows vehicle's status.
 */ 
void showVehicleStatus(void){
  std::string temp;
  if(my_auv_.is_traversing_){
    temp = "Traversing and ";
    switch(mode_){
      case ROV_MODE:  temp += "ROV";
                      break;
      case AUV_MODE:  temp += "AUV";
                      break;
      case IDLE_STATE: temp += "IDLE";
                      break;               
    }
  }
  else temp = "Not traversing. By entering traversing mode, you enter IDLE state.";
  ROS_INFO_STREAM("\nMode: " << temp);
  ROS_INFO_STREAM("\n********* Setpoints ********* " << std::endl
                 << "Surge thrust: " << my_auv_.set_xyz_[0] << std::endl
                 << "Sway thrust: " << my_auv_.set_xyz_[1] << std::endl
                 << "Heave setpoint: " << my_auv_.set_xyz_[0] << std::endl
                 << "Yaw setpoint: " << my_auv_.set_orient_.yaw << std::endl);
  my_auv_.showPidConfig();
}

/**
 * @brief Displays descriptions of instructions recognisable by the
 * ROS node.
 */
void displayHelp(void)
{
  std::cout << "\nUse the following commands: " << std::endl
            << "t: Start/Stop traversing" << std::endl
            << "r: Start/Stop ROV mode (default: idle state)" << std::endl
            << "R: Start/Stop AUV mode (default: idle state)" << std::endl
            << "--- Motion Commands ---" << std::endl
            << "W: Heave down by " << HEAVE_PLUS << " m" << std::endl
            << "Q: Heave up by " << HEAVE_MINUS << " m" << std::endl
            << "d: Positive yaw by " << YAW_PLUS << " deg" << std::endl
            << "a: Negative yaw by " << YAW_MINUS << " deg" << std::endl
            << "w: Surge+ by " << SURGE_PLUS << std::endl
            << "s: Surge- by " << SURGE_MINUS << std::endl
            << "e: Sway+ by " << SWAY_PLUS << std::endl
            << "q: Sway- by " << SWAY_MINUS << std::endl
            << "S: Stop thrusters (stops only surge and sway in traversing mode)" << std::endl
            << "--- Parameter settings ---" << std::endl
            << "p: Show all PID parameters" << std::endl
            << "P: Refresh PID configuration" << std::endl
            << "-----" << std::endl
            << "v: Show vehicle status" << std::endl
            << "Z: Display help" << std::endl
            << "X: Quit\n"
            << std::endl;
}


int main(int argc, char** argv)
{
  // initializing ROS node
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;

  // register a signal handler to ensure that node is exited properly
  signal(SIGINT, signalHandler);

  // initialize the underwater vehicle
  my_auv_.initUWV(nh, 40.0, 0.4);

  displayHelp();

  // begin separate thread to read from the keyboard
  std::thread reading_input_thread(readInput);
  reading_input_thread.detach();

  // begin the PID thread
  std::thread pid_thread;

  // main loop
  while (do_not_quit_ && ros::ok())
  {
    if (my_auv_.is_traversing_)
    { 

      /****** define autonomous operation ****/
      if(mode_ == AUV_MODE)
      { 
        // resetting flags
        obstacle_found_ = false;
        task_complete_ = false;
        while (ros::ok() && !task_complete_)
        {
          // Looking for object in the current field of view
          task_1::detectObstacle();

          ROS_INFO_STREAM("Obstacle found: " << obstacle_found_);

          if (!obstacle_found_)
          {
            task_1::scanObstacle();
          }
          else
          {
            move::doSurge(28, 10);
            my_auv_.allStop();
            task_complete_ = true;
            mode_ = IDLE_STATE;
            ROS_INFO_STREAM("Task completed! Exiting AUV mode. Entering idle state.");
          }
          ros::spinOnce();
        }
      }
      /****** end autonomous operations ****/

      if(!pid_thread.joinable())
      {
        pid_thread = std::thread([&](){
          my_auv_.reuseThread();
          my_auv_.run();
        });
      }
    }
    else
    {
      if(!my_auv_.stopRequested())
      {
        my_auv_.stop();
      }
      if(pid_thread.joinable())
      {
        pid_thread.join();
      }
    }
    ros::spinOnce();
  }
  if(!my_auv_.stopRequested()){
    my_auv_.stop();
  }
  if(pid_thread.joinable()){
    pid_thread.join();
  }
  ROS_INFO_STREAM("Sayonara.");
}

// signal handler
void signalHandler(int signum)
{
  ROS_INFO_STREAM("[!] Press X to quit\n");
}