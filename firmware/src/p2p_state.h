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

#include "configblock.h"

#include "debug.h"

#include "stabilizer_types.h"

#include "network_evaluate_tof.h"

// #define ENABLE_NEIGHBOR_REL_VEL

// How many ms between each broadcast 
#define BROADCAST_PERIOD_MS 10

typedef struct {
    uint8_t id;

    //Position Estimates
    float x_pos;
    float y_pos;
    float z_pos;

    // Velocity Estimates
    #ifdef ENABLE_NEIGHBOR_REL_VEL
        float x_vel;
        float y_vel;
        float z_vel;
    #endif

} message_state_t;

// #define NETWORK_TOPOLOGY {.size = (uint8_t) NUM_NEIGHBORS, .devices_ids = {0, 1, 2} }

/**
 * @brief Initializes the token ring network protocol
 * 
 * @param topology
 */
void network_init();

/**
 * @brief Forms the packet and sends to the other robots
 * 
 * @param state 
 * @return true 
 * @return false 
 */
void broadcastState(message_state_t tx_message);

/**
 * @brief Updates neighboar array with the 2 closests drones.
 * 
 * @param neighbor_array 
 */
// void updateNeighborData(dtrTopology topology, uint8_t my_id, float *rel_distance, const state_t *state, float *neighbor_array);

float get_dist(float *p1, float *p2);

void updateNeighborInputs(const state_t *state, float *neighbor_inputs);
// bool valid_packet(dtrPacket* packet, dtrTopology topology);

// Loggging variables for both broadcast
float getX();
float getY();
float getZ();
float getVx();
float getVy();
float getVz();