/**
 * @file p2p_state.h
 * @author Darren Chiu (chiudarr@usc.edu)
 * @brief 
 * @version 0.1
 * @date 2024-04-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "debug.h"

#include "token_ring.h"
#include "DTR_p2p_interface.h"
#include "stabilizer_types.h"

#include "network_evaluate_tof.h"

#define ENABLE_NEIGHBOR_REL_VEL

#ifdef ENABLE_NEIGHBOR_REL_VEL
    //Using relative position and velocity
    static uint8_t data_size = sizeof(float)*6;
#else
    // Using ONLY relative position
    static uint8_t data_size = sizeof(float)*3;
#endif

#define NETWORK_TOPOLOGY {.size = (uint8_t) NUM_NEIGHBORS, .devices_ids = {0, 1} }

/**
 * @brief Initializes the token ring network protocol
 * 
 * @param topology
 */
uint8_t network_init(dtrTopology topology, float *rel_distance);

/**
 * @brief This handles packet queues
 * 
 * @param p 
 */
void p2pcallBackHandler(P2PPacket *p);

/**
 * @brief Forms the packet and sends to the other robots
 * 
 * @param state 
 * @return true 
 * @return false 
 */
bool broadcastState(uint8_t source_id, const state_t *state);

/**
 * @brief Updates neighboar array with the 2 closests drones.
 * 
 * @param neighbor_array 
 */
void updateNeighborData(dtrTopology topology, uint8_t my_id, float *rel_distance, const state_t *state, float *neighbor_array);

float get_dist(float *p1, float *p2);

bool valid_packet(dtrPacket* packet, dtrTopology topology);