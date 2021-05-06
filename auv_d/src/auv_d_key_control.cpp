// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Misc Libraries
#include <termios.h>
#include <thread>

// defining publishers
ros::Publisher joint_state_pub;

// Control variables
sensor_msgs::JointState set_thrust;
float cur_set_point[2];  // Set points from user: 0 - port; 1 - star;
float prev_set_point[2];

// Misc variables
static struct termios old, current;
bool do_not_quit = true;

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
void read_input(void)
{
  char input;

  while (do_not_quit)
  {
    input = getch();

    switch (input)
    {
      case 'q':
        do_not_quit = false;
        break;
      case 's':
        cur_set_point[0] = cur_set_point[0] + 0.1;
        break;
      case 'a':
        cur_set_point[0] = cur_set_point[0] - 0.1;
        break;
      case 'l':
        cur_set_point[1] = cur_set_point[1] + 0.1;
        break;
      case 'k':
        cur_set_point[1] = cur_set_point[1] - 0.1;
        break;
    }
  }
}

/* Sending set point to Gazebo */
void send_effort_values(void)
{
  if (prev_set_point[0] != cur_set_point[0] || prev_set_point[1] != cur_set_point[1])
  {
    set_thrust.effort[0] = cur_set_point[0];
    set_thrust.effort[1] = cur_set_point[1];

    set_thrust.header.stamp = ros::Time::now();

    joint_state_pub.publish(set_thrust);

    ros::spinOnce();
    prev_set_point[0] = cur_set_point[0];
    prev_set_point[1] = cur_set_point[1];

    std::cout << "thruster_port: " << cur_set_point[0] << std::endl
              << "thruster_star: " << cur_set_point[1] << std::endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "auv_d_key_control");
  ros::NodeHandle nh;
  ros::Rate rate(2);

  set_thrust.name.resize(2);
  set_thrust.effort.resize(2);

  // advertising the publishers
  joint_state_pub = nh.advertise<sensor_msgs::JointState>("/auv_d/thruster_command", 1000);

  // setting default values
  for (int i = 0; i < 2; i++)
  {
    cur_set_point[i] = 0;
    prev_set_point[i] = 0;
  }
  set_thrust.name[0] = "thruster_port";
  set_thrust.name[1] = "thruster_star";
  set_thrust.effort[0] = 0.0;
  set_thrust.effort[1] = 0.0;

  std::cout << "\nUse the following commands: " << std::endl
            << "s: increase port thruster's effort by 0.1 N" << std::endl
            << "a: decrease port thruster's effort by 0.1 N" << std::endl
            << "l: increase starboard thruster's effort by 0.1 N" << std::endl
            << "k: decrease starboard thruster's effort by 0.1 N" << std::endl
            << "q: to quit\n"
            << std::endl;

  std::thread reading_input_thread(read_input);
  reading_input_thread.detach();

  while (do_not_quit && ros::ok())
  {
    send_effort_values();
  }

  std::cout << "Successfully ended the program." << std::endl;
}
