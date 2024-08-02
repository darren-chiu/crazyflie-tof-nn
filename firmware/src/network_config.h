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

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"

/**
 * BASIC NETWORK SETTINGS
 */

// Enable print statements
// #define DEBUG_LOCALIZATION

// Defines the dynamics dimmensions
#define STATE_DIM 18

#define REL_VEL true
#define REL_OMEGA true
#define REL_XYZ true
#define REL_ROT true

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
#define OBST_MAX 1.0f
// Sets at which height to accept ToF readings.
#define SAFE_HEIGHT 0.0f
static uint16_t tof_addresses[NUM_SENSORS] = {0x50, 0x66, 0x76, 0x86};



/**
 * MULTI DRONE SETTINGS
 */

// Enable Multi Drone
// 
// #define MULTI_DRONE_ENABLE

// Number of TOTAL drones (this includes yourself)
#define NUM_DRONES 4
//Size of the neighbor encoder for the network
#define NEIGHBORS 2
#define NBR_OBS_DIM 3
// How many ms between each broadcast 
#define BROADCAST_PERIOD_MS 100
// #define ENABLE_NEIGHBOR_REL_VEL

#define ABLATE_NEIGHBOR true

// Communication Port
#define P2P_PORT 5


#endif