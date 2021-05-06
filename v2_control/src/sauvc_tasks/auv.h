#ifndef AUV_H
#define AUV_H

#include "config_AUV.h" 	// This should be called in the very beginning only
#define ROV_MODE 	0
#define AUV_MODE 	1
#define IDLE_STATE 	2

// ROS libraries
#include <ros/ros.h>

// Control libraries
#include "soft_uwv.h"

// Misc libraries
#include <cmath>
#include <thread>
#include <csignal>
#include "terminal_getch.h"

// Instance of the underwater vehicle (UWV)
SoftUWV my_auv_;

// Task completion boolean
bool obstacle_found_;
bool task_complete_;

// Vars to read user cmds
bool do_not_quit_ = true;
short mode_ = IDLE_STATE;

// Functions to read user input
void displayHelp(void);
void readInput(void);

// Misc functions
void showVehicleStatus(void);
void signalHandler(int signum);

#endif  // AUV_H