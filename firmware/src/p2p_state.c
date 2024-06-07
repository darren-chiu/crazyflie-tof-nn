#include "network_evaluate_tof.h"
#include "p2p_state.h"
#include "log.h"
#include "task.h"
#include "radiolink.h"


#define P2P_PORT 5

static uint8_t self_id;
// Tracks the distances between self and N drones (N=NEIGHBORS)
// Index 3 represents the distance from YOU and Drone 3. Assuming you are not drone 3. 
static float rel_distance[NUM_DRONES] = { __FLT_MAX__};
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

    rel_distance[rx_message.id] = dist;
    neighbor_array[rx_message.id] = rx_message;


    count++;
    #ifdef DEBUG_COMMUNICATION
        if (count == 500){
        DEBUG_PRINT("Received CF[%u]: (%f, %f, %f)\n", rx_message.id, rx_message.x_vel, rx_message.y_vel, rx_message.z_vel);
            count = 0;
        }
    #endif
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
}

void network_init() {
    //================================== Basic P2P Implementaiton ============================================
    initLogIds();

    p2pRegisterCB(p2pcallbackHandler);
    uint64_t address = configblockGetRadioAddress();
    self_id = (uint8_t)((address) & 0x00000000ff);
}

void broadcastState(message_state_t tx_message) {
    // ==================================== Basic P2P Implementation ======================================= 
    static P2PPacket packet;
    tx_message.id = self_id;
    uint8_t packet_size = sizeof(message_state_t);
    packet.port = P2P_PORT;
    memcpy(packet.data, &tx_message, packet_size);

    packet.size = packet_size;
    radiolinkSendP2PPacketBroadcast(&packet);
}

// Sqrt is an iterative operation. The distance returned is not a true "distance" but just a measure.
float get_dist(float *p1, float *p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];

    return dx*dx + dy*dy + dz*dz;
}

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