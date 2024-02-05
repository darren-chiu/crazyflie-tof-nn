#ifndef __NETWORK_EVALUATE_H__
#define __NETWORK_EVALUATE_H__

#include <math.h>
#include <stdbool.h>
#include "debug.h"

#define OBST_DIM 8 // ignore this if you are using a multi-agent policy without obstacle encoder or just a single agent policy


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
void obstacleEmbedder(const float obstacle_inputs[OBST_DIM]);

#endif