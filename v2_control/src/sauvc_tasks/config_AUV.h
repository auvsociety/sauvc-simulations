#ifndef CONFIG_AUV_H
#define CONFIG_AUV_H

/* ROV mode configuration */
// Absolute step sizes (units: m, deg)
#define HEAVE_PLUS 		0.1
#define HEAVE_MINUS 	0.1
#define YAW_PLUS 		10
#define YAW_MINUS 		10
#define SURGE_PLUS		0.5
#define SURGE_MINUS		0.5
#define SWAY_PLUS		0.5
#define SWAY_MINUS		0.5

/* AUV mode configuration */
#define MOVE_SPIN_RATE 100
#define GET_COUNTDOWN_TICKS(x) x*MOVE_SPIN_RATE

// Node name
#define NODE_NAME "auv_ctrl"

#endif