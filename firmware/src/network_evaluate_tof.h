#ifndef __NETWORK_EVALUATE_TOF_H__
#define __NETWORK_EVALUATE_TOF_H__

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"

// When defined, uses 4x4 as ToF input with corresponding controller
#define ENABLE_4X4_CONTROLLER


#ifdef ENABLE_4X4_CONTROLLER
	#define OBST_DIM 16
#else
	#define OBST_DIM 32
#endif

/**
 * @brief Defines the output thrusts from the neural network.
 * 
 */
typedef struct control_t_n {
	float thrust_0; 
	float thrust_1;
	float thrust_2;
	float thrust_3;	
} control_t_n;

/**
 * @brief Propogates the neural network to generate thrust values.
 * 
 * @param control_n 
 * @param state_array 
 * @param obstacle_embeds 
 */
void networkEvaluate(control_t_n *control_n, const float *state_array);

/**
 * @brief Encodes the input array of ToF readings.
 * 
 */
void obstacleEmbedder(float obstacle_inputs[OBST_DIM]);

#endif