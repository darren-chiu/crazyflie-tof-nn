#ifndef NETWORK_EVALUATE_TOF_H
#define NETWORK_EVALUATE_TOF_H
#include "network_config.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"

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

/**
 * @brief Encodes the neighbor array.
 * 
 */
void neighborEmbedder(float neighbor_inputs[NEIGHBORS*NBR_OBS_DIM]);

#endif