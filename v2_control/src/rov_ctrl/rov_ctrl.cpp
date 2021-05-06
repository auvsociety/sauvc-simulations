#include "rov_ctrl.h"

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
      case 'Z':
        displayHelp();
        break;

      // Start/stop traversing
      case 't':
        if (v2_.is_traversing_)
        {
          v2_.is_traversing_ = false;
          ROS_INFO_STREAM("Exiting traversing mode.");
        }
        else
        {
          v2_.is_traversing_ = true;
          ROS_INFO_STREAM("Entering traversing mode...");
        }
        break;

      // Stop thrusters
      case 'S':
        v2_.allStop();
        v2_.set_xyz_[0] = v2_.set_xyz_[1] = 0;
        ROS_INFO_STREAM("Sending thruster command: ZERO to all thrusters.");
        break;

      // Heave
      case 'W':
        v2_.set_xyz_[2] += HEAVE_PLUS;
        ROS_INFO_STREAM("Set depth: " << v2_.set_xyz_[2]);
        break;
      case 'Q':
        v2_.set_xyz_[2] -= HEAVE_MINUS;
        if (v2_.set_xyz_[2] < 0)
          v2_.set_xyz_[2] = 0;         // cannot give set point that is above water level
        ROS_INFO_STREAM("Set depth: " << v2_.set_xyz_[2]);
        break;

      // Yaw
      case 'd':
        v2_.set_orient_.yaw += YAW_PLUS;
        if(v2_.set_orient_.yaw >= 180.01){
					v2_.set_orient_.yaw = -(360 - v2_.set_orient_.yaw);
				}
        ROS_INFO_STREAM("Set heading: " << v2_.set_orient_.yaw);
        break;
      case 'a':
        v2_.set_orient_.yaw -= YAW_MINUS;
        if(v2_.set_orient_.yaw <= -179.9){
					v2_.set_orient_.yaw = 180;
				}
        ROS_INFO_STREAM("Set heading: " << v2_.set_orient_.yaw);
        break;

      // Refresh PID configuration
      case 'P':
        v2_.is_traversing_ = false;
        ROS_INFO_STREAM("Exiting traversing mode.");
        v2_.reconfigPid(true, true);
        ROS_INFO_STREAM("PID configuration reloaded. Entering traversing mode.");
        v2_.is_traversing_ = true;
        break;
      // Display all PID parameters
      case 'p': v2_.showPidConfig();
                break;
      
      // Surge
      case 'w':
        v2_.set_xyz_[0] += SURGE_PLUS;
        ROS_INFO_STREAM("Surging effort: " << v2_.set_xyz_[0]);
        break;
      case 's':
        v2_.set_xyz_[0] -= SURGE_MINUS;
        ROS_INFO_STREAM("Surging effort: " << v2_.set_xyz_[0]);
        break;

      // Sway
      case 'e':
        v2_.set_xyz_[1] += SWAY_PLUS;
        ROS_INFO_STREAM("Sway effort: " << v2_.set_xyz_[1]);
        break;
      case 'q':
        v2_.set_xyz_[1] -= SWAY_MINUS;
        ROS_INFO_STREAM("Sway effort: " << v2_.set_xyz_[1]);
        break;
    }
  }
}

/**
 * @brief Displays descriptions of instructions recognisable by the
 * ROS node.
 */
void displayHelp(void)
{
  std::cout << "\nUse the following commands: " << std::endl
            << "t: Start/Stop traversing" << std::endl
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
  v2_.initUWV(nh, 40.0, 0.4);

  displayHelp();

  // begin separate thread to read from the keyboard
  std::thread reading_input_thread(readInput);
  reading_input_thread.detach();

  // initial values
  v2_.set_xyz_[0] = 0;
  v2_.set_xyz_[1] = 0;
  v2_.set_xyz_[2] = 0.7;
  v2_.set_orient_.yaw = 0.0;
  ROS_INFO_STREAM("Initial depth value: " << v2_.set_xyz_[2] << " | Initial heading: " << v2_.set_orient_.yaw);

  // begin the PID thread
  std::thread pid_thread;

  // main loop
  while (do_not_quit_ && ros::ok())
  {
    if (v2_.is_traversing_)
    {
      if(!pid_thread.joinable())
      {
        pid_thread = std::thread([&](){
          v2_.reuseThread();
          v2_.run();
        });
      }
    }
    else
    {
      if(!v2_.stopRequested())
      {
        v2_.stop();
      }
      if(pid_thread.joinable())
      {
        pid_thread.join();
      }
    }
    ros::spinOnce();
  }
  if(!v2_.stopRequested()){
    v2_.stop();
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