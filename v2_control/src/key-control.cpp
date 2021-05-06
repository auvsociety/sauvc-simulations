// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Misc Libraries
#include <termios.h>
#include <thread>
#include <csignal>

#define THRUSTER_NUM 6

// defining publishers
ros::Publisher joint_state_pub;

// Control variables
sensor_msgs::JointState set_thrust;
float cur_effort[THRUSTER_NUM];
float prev_effort[THRUSTER_NUM];

// Misc variables
static struct termios old, current;
bool do_not_quit = true;

// Control variable
bool set_position = false;
bool efforts_changed = false;

// signal handler
void signalHandler(int signum)
{
  std::cout<<"[!] Press x to quit\n";
}

// Prototype
void displayHelp(void);

/* Receiving set point from the user */
void initTermios(void)
{
  tcgetattr(0, &old);              /* grab old terminal i/o settings */
  current = old;                   /* make new settings same as old settings */
  current.c_lflag &= ~ICANON;      /* disable buffered i/o */
  current.c_lflag &= ~ECHO;        /* set no echo mode */
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}
char getch(void)
{
  char ch;
  initTermios();
  ch = getchar();
  resetTermios();
  return ch;
}

void readInput(void)
{
  char input;

  while (do_not_quit)
  {
    input = getch();

    switch (input)
    {
      case 'x':
        do_not_quit = false;
        break;
      case 'h':
        displayHelp();
        break;
      case 'w':
        cur_effort[2] += 0.1;
        cur_effort[3] += 0.1;
        break;
      case 'q':
        cur_effort[2] -= 0.1;
        cur_effort[3] -= 0.1;
        break;
      case 's':
        cur_effort[2] = 2.2;
        cur_effort[3] = 2.2;
        cur_effort[0] = 0;
        cur_effort[1] = 0;
        cur_effort[4] = 0;
        cur_effort[5] = 0;
        break;
      case 'i':
        cur_effort[0] += 0.1;
        cur_effort[1] += 0.1;
        cur_effort[4] += 0.1;
        cur_effort[5] += 0.1;
        break;
      case 'k':
        cur_effort[0] -= 0.1;
        cur_effort[1] -= 0.1;
        cur_effort[4] -= 0.1;
        cur_effort[5] -= 0.1;
        break;
      case 'j':
        cur_effort[0] += 0.1;
        cur_effort[1] += -0.1;
        cur_effort[4] += 0.1;
        cur_effort[5] += -0.1;
        break;
      case 'l':
        cur_effort[0] -= 0.1;
        cur_effort[1] -= -0.1;
        cur_effort[4] -= 0.1;
        cur_effort[5] -= -0.1;
        break;
      case 'o':
        cur_effort[0] += 0.1;
        cur_effort[1] += -0.1;
        cur_effort[4] += -0.1;
        cur_effort[5] += 0.1;
        break;
      case 'u':
        cur_effort[0] -= 0.1;
        cur_effort[1] -= -0.1;
        cur_effort[4] -= -0.1;
        cur_effort[5] -= 0.1;
        break;
    }
  }
}

/* Sending efforts to Gazebo */
void sendEffortValues(void)
{
  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    if (prev_effort[i] != cur_effort[i])
    {
      efforts_changed = true;
      break;
    }
  }
  if (efforts_changed)
  {
    for (int i = 0; i < THRUSTER_NUM; i++)
    {
      prev_effort[i] = cur_effort[i];
      set_thrust.effort[i] = cur_effort[i];
    }
    set_thrust.header.stamp = ros::Time::now();

    joint_state_pub.publish(set_thrust);

    ros::spinOnce();
    prev_effort[2] = cur_effort[2];
    prev_effort[3] = cur_effort[3];

    std::cout << cur_effort[0] << " " << cur_effort[1] << " | " << cur_effort[2] << " " << cur_effort[3] << " | "
              << cur_effort[4] << " " << cur_effort[5] << std::endl;
    efforts_changed = false;
  }
}

void displayHelp(void)
{
  std::cout << "\nUse the following commands:: " << std::endl
            << "w: Heave+" << std::endl
            << "q: Heave-" << std::endl
            << "j: Yaw+" << std::endl
            << "l: Yaw-" << std::endl
            << "i: Surge+" << std::endl
            << "k: Surge-" << std::endl
            << "o: Sway+" << std::endl
            << "u: Sway-" << std::endl
            << "s: Set position" << std::endl
            << "h: Display help" << std::endl
            << "x: Quit\n"
            << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_control");
  ros::NodeHandle nh;
  
  // register a signal handler to ensure that node is exited properly
  signal(SIGINT, signalHandler);

  set_thrust.name.resize(THRUSTER_NUM);
  set_thrust.effort.resize(THRUSTER_NUM);

  // advertising the publishers
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/auv_v2/thruster_command", 1000);

  // setting default values
  set_thrust.name[0] = "f_port";
  set_thrust.name[1] = "f_star";
  set_thrust.name[2] = "m_port";
  set_thrust.name[3] = "m_star";
  set_thrust.name[4] = "b_port";
  set_thrust.name[5] = "b_star";

  for (int i = 0; i < THRUSTER_NUM; i++)
  {
    cur_effort[i] = 0;
    prev_effort[i] = 0;
    set_thrust.effort[i] = 0.0;
  }

  displayHelp();

  std::thread reading_input_thread(readInput);
  reading_input_thread.detach();

  while (do_not_quit && ros::ok())
  {
    sendEffortValues();
  }
}
