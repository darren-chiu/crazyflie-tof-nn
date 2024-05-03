#include "network_evaluate_tof.h"
#include "p2p_state.h"
#include "log.h"
#include "task.h"
#include "radiolink.h"


#define P2P_PORT 5
// Neighbor Observations
// #ifdef MULTI_DRONE_ENABLE
// static dtrTopology topology = NETWORK_TOPOLOGY;
static uint8_t self_id;
// Tracks the distances between self and N drones (N=NEIGHBORS)
// Index 3 represents the distance from YOU and Drone 3. Assuming you are not drone 3. 
static float rel_distance[NUM_DRONES] = { __FLT_MAX__};
// static bool isPeerStale = false;
// This would track neighbor positions
static message_state_t neighbor_array[NUM_DRONES];

static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdStateEstimateVx;
static logVarId_t logIdStateEstimateVy;
static logVarId_t logIdStateEstimateVz;

static int count = 0;
static int index1;
static int index2;

static void p2pcallbackHandler(P2PPacket *p){
	// If the packet is a DTR service packet, then the handler will handle it.
	// It returns true if the packet was handled.
    // if (!dtrP2PIncomingHandler(p)){
	// 	// If packet was not handled from DTR , then it is a normal packet 
	// 	// that user has to handle. 
    //     return;
	// }


    // ========================== Basic P2P Implementation ===============================
    if (p->port != P2P_PORT){
        DEBUG_PRINT("Wrong port %u\n", p->port);
        return;
    }
    message_state_t rx_message;
    memcpy(&rx_message, p->data, sizeof(message_state_t));

    float other_pos[3] = {rx_message.x_pos, rx_message.y_pos, rx_message.z_pos};
    float my_pos[3] = {getX(), getY(), getZ()};

    float dist = get_dist(other_pos, my_pos);

    #ifdef DEBUG_COMMUNICATION
        DEBUG_PRINT("Received ID=%u: (%f, %f, %f)\n", rx_message.id, other_pos[0], other_pos[1], other_pos[2]);
    #endif
    rel_distance[rx_message.id] = dist;
    neighbor_array[rx_message.id] = rx_message;

    //Check to see if the robots are closer to yourself.
    index1 = 0;
    index2 = 1;
    int min1 = rel_distance[index1];
    int min2 = rel_distance[index2];

    if (min1 > min2){
        int temp = min1;
        min1 = min2;
        min2 = temp;

        index1 = 1;
        index2 = 0;
    }

    // for (int i=0; i < NUM_DRONES; i++) {
    //     if (dist < rel_distance[i]) {
    //         for (int j=0;j<6;j++) {
    //             rel_distance[i] = dist;
    //             neighbor_array[i] = rx_message;
    //         }
    //     }
    // }

    for (int i = 2; i < NUM_DRONES; i++) {
        if (rel_distance[i] < min1) {
            int temp = min1;
            min1 = rel_distance[i];
            min2 = temp;
            index2 = index1;
            index1 = i;
        } else if (rel_distance[i] < min2) {
            min2 = rel_distance[i];
            index2 = i;
        }
    }
}

void updateNeighborInputs(const state_t *state, float *neighbor_inputs) {
    int indexarray[2] = {index1, index2};
    count++;
    for (int i=0;i<NEIGHBORS;i++) {
        neighbor_inputs[0 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].x_pos - state->position.x;
        neighbor_inputs[1 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].y_pos - state->position.y;
        neighbor_inputs[2 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].z_pos - state->position.z;
        #ifdef ENABLE_NEIGHBOR_REL_VEL
            neighbor_inputs[3 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].x_vel - state->velocity.x;
            neighbor_inputs[4 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].y_vel - state->velocity.y;
            neighbor_inputs[5 + (i*NBR_OBS_DIM)] = neighbor_array[indexarray[i]].z_vel - state->velocity.z;
        #endif
    }

    // if (count == 500) {
    //     DEBUG_PRINT("ID: %u (%f, %f, %f), (%f, %f, %f)\n", self_id, neighbor_inputs[0], neighbor_inputs[1], neighbor_inputs[2], neighbor_inputs[3], neighbor_inputs[4], neighbor_inputs[5]);
    //     // DEBUG_PRINT("ID: %u (%f, %f, %f), (%f, %f, %f)\n", self_id, neighbor_array[0].x_pos, neighbor_array[0].y_pos, neighbor_array[0].z_pos, neighbor_array[1].x_pos, neighbor_array[1].y_pos, neighbor_array[1].z_pos);
    //     count = 0;
    // }

    // this is really lazy
    // neighbor_inputs[0] = neighbor_array[0].x_pos - state->position.x;
    // neighbor_inputs[1] = neighbor_array[0].y_pos - state->position.y;
    // neighbor_inputs[2] = neighbor_array[0].z_pos - state->position.z;
    // neighbor_inputs[3] = neighbor_array[0].x_vel - state->velocity.x;
    // neighbor_inputs[4] = neighbor_array[0].y_vel - state->velocity.y;
    // neighbor_inputs[5] = neighbor_array[0].z_vel - state->velocity.z;

    // neighbor_inputs[6] = neighbor_array[1].x_pos - state->position.x;
    // neighbor_inputs[7] = neighbor_array[1].x_pos - state->position.x;
    // neighbor_inputs[8] = neighbor_array[1].x_pos - state->position.x;
    // neighbor_inputs[9] = neighbor_array[1].x_pos - state->position.x;
    // neighbor_inputs[10] = neighbor_array[1].x_pos - state->position.x;
    // neighbor_inputs[11] = neighbor_array[1].x_pos - state->position.x;
}

void network_init() {
    //================================== DTR P2P Implementaiton ============================================
    // dtrEnableProtocol(topology);
	// vTaskDelay(2000);
	// p2pRegisterCB(p2pcallBackHandler);
    // uint8_t my_id = dtrGetSelfId();

    // //Initialize your own relative distance to a high value.
    // for (uint8_t i=0;i<NUM_NEIGHBORS;i++) {
    //     if (i != my_id) {
    //         rel_distance[i] = 10000.0f;
    //     } 
    // }
    // #ifdef DEBUG_COMMUNICATION
    //     DEBUG_PRINT("Network Topology: %d:", topology.size);
    //     for (int i = 0; i < topology.size; i++){
    //         DEBUG_PRINT("%d ", topology.devices_ids[i]);
    //     }
    //     DEBUG_PRINT("\n");
    //     DEBUG_PRINT("My ID: %d\n", my_id);
    // #endif


    //================================== Basic P2P Implementaiton ============================================
    initLogIds();

    p2pRegisterCB(p2pcallbackHandler);
    uint64_t address = configblockGetRadioAddress();
    self_id = (uint8_t)((address) & 0x00000000ff);
}

void broadcastState(message_state_t tx_message) {
    // ================================= DTR P2P Implementation ============================================
    // dtrPacket statePacket;
    // statePacket.dataSize = data_size;
    // statePacket.packetSize = DTR_PACKET_HEADER_SIZE + data_size;
    // statePacket.sourceId = self_id;
    // statePacket.targetId  = 0xFF;
    // statePacket.messageType = DATA_FRAME;
    
    // #ifdef ENABLE_NEIGHBOR_REL_VEL
    //     float prep_buffer[6];
    //     prep_buffer[0] = state->position.x;
    //     prep_buffer[1] = state->position.y;
    //     prep_buffer[2] = state->position.z;
        
    //     prep_buffer[3] = state->velocity.x;
    //     prep_buffer[4] = state->velocity.y;
    //     prep_buffer[5] = state->velocity.z;

    //     // Split the float representation into 4 indexes of bytes.
    //     // Packet size is number of bytes. uint8_t is one byte.
    //     memcpy(statePacket.data, prep_buffer, data_size);
    // #else
    //     float prep_buffer[3];
    //     prep_buffer[0] = state->position.x;
    //     prep_buffer[1] = state->position.y;
    //     prep_buffer[2] = state->position.z;

    //     // Split the float representation into 4 indexes of bytes.
    //     memcpy(send_buffer, prep_buffer, sizeof(data_size));
    // #endif

    // bool msg_status;

    // msg_status = dtrSendPacket(&statePacket);

    // return msg_status;

    // ==================================== Basic P2P Implementation ======================================= 
    static P2PPacket packet;
    tx_message.id = self_id;
    packet.port = P2P_PORT;
    memcpy(packet.data, &tx_message, sizeof(message_state_t));

    packet.size = sizeof(message_state_t);
    radiolinkSendP2PPacketBroadcast(&packet);
}

// void updateNeighborData(float *rel_distance, const state_t *state, float *neighbor_array) {
    // uint8_t other_id = received_packet.sourceId;

    // #ifdef ENABLE_NEIGHBOR_REL_VEL
    //     float other_pos[6];
    // #else
    //     float other_pos[3];
    // #endif
    // memcpy(other_pos, received_packet.data, data_size);

    // float my_pos[3] = {state->position.x, state->position.y, state->position.z};
    // float dist = get_dist(other_pos, my_pos);
    // #ifdef DEBUG_COMMUNICATION
    //     DEBUG_PRINT("Received ID=%d: (%f, %f, %f)\n", other_id, other_pos[0], other_pos[1], other_pos[2]);
    // #endif
    // // Update distances array
    // rel_distance[other_id] = dist;

    // //Check to see if the robots are closer to yourself.
    // for (int i=0; i < NEIGHBOR_ATTENTION; i++) {
    //     float current_neighbor_pos[3] = {neighbor_array[0], neighbor_array[1], neighbor_array[2]};
    //     if (dist < get_dist(my_pos, current_neighbor_pos)) {
    //         #ifdef ENABLE_NEIGHBOR_REL_VEL
    //             for (int j=0;j<6;j++) {
    //                 neighbor_array[j+(i*6)] = other_pos[j];
    //             }
    //         #else
    //             for (int j=0;j<3;j++) {
    //                 neighbor_array[j+(i*3)] = other_pos[j];
    //             }
    //         #endif

    //         //Don't bother checking the other ones
    //         return;
    //     }
    // }
// }   

// Sqrt is an iterative operation. The distance returned is not a true "distance" but just a measure.
float get_dist(float *p1, float *p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];

    return dx*dx + dy*dy + dz*dz;
}

// bool valid_packet(dtrPacket* packet, dtrTopology topology) {
//     for (int i=0; i < topology.size; i++) {
//         if ((packet->sourceId == topology.devices_ids[i]) & (packet->sourceId != dtrGetSelfId())) return true;
//     }

//     return false;
// }

void initLogIds(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateVx = logGetVarId("stateEstimate", "vx");
    logIdStateEstimateVy = logGetVarId("stateEstimate", "vy");
    logIdStateEstimateVz = logGetVarId("stateEstimate", "vz");
}

float getX() { return (float) logGetFloat(logIdStateEstimateX); }
float getY() { return (float) logGetFloat(logIdStateEstimateY); }
float getZ() { return (float) logGetFloat(logIdStateEstimateZ); }
float getVx() { return (float) logGetFloat(logIdStateEstimateVx); }
float getVy() { return (float) logGetFloat(logIdStateEstimateVy); }
float getVz() { return (float) logGetFloat(logIdStateEstimateVz); }