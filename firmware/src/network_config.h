/**
 * @file network_config.h
 * @author Darren Chiu (chiudarr@usc.edu)
 * @brief Contains necessary parameters for evaluating RL policies on the crazyflie.
 * @version 1.0
 * @date 2024-06-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef NETWORK_CONFIG_H
#define NETWORK_CONFIG_H
/**
 * BASIC NETWORK SETTINGS
 */
// Enable print statements for communication
// #define DEBUG_COMMUNICATION

// Enable print statements
// #define DEBUG_LOCALIZATION

// Defines the dynamics dimmensions
#define STATE_DIM 18

#define REL_VEL true
#define REL_OMEGA true
#define REL_XYZ true
#define REL_ROT true

// Toggle input normalization
#define ENABLE_BATCH_NORM
// Used for network input normalization
#define NORM_EPS 0.00001

/**
 * OBSERVATION SETTINGS
 */
// When defined, enables the ToF module.
#define TOF_ENABLE

// When defined, uses 4x4 as ToF input with corresponding controller
// #define ENABLE_4X4_CONTROLLER 
#ifdef ENABLE_4X4_CONTROLLER
	#define OBST_DIM 16	
#else
	#define OBST_DIM 32
#endif
// The number of ToF sensors to be used
#define NUM_SENSORS 4

// Obstacle Avoidance Parameters
#define OBST_MAX 2.0f
// Sets at which height to accept ToF readings.
#define SAFE_HEIGHT 0.0f

//The order which allows us to use the ToF in clockwise order
static uint8_t pin_order[4] = {0, 2, 1, 3}; 



/**
 * MULTI DRONE SETTINGS
 */
//Enable Multi Drone
// #define MULTI_DRONE_ENABLE

// Number of TOTAL drones (this includes yourself)
#define NUM_DRONES 3
//Size of the neighbor encoder for the network
#define NEIGHBORS 2
#define NBR_OBS_DIM 3
// How many ms between each broadcast 
#define BROADCAST_PERIOD_MS 10
// #define ENABLE_NEIGHBOR_REL_VEL

// Communication Port
#define P2P_PORT 5


#endif