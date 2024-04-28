#include "network_evaluate_tof.h"
#include "p2p_state.h"

uint8_t network_init(dtrTopology topology, float *rel_distance) {
    dtrEnableProtocol(topology);
	vTaskDelay(2000);
	p2pRegisterCB(p2pcallBackHandler);
    uint8_t my_id = dtrGetSelfId();

    //Initialize your own relative distance to a high value.
    for (uint8_t i=0;i<NUM_NEIGHBORS;i++) {
        if (i != my_id) {
            rel_distance[i] = 10000.0f;
        } 
    }
    #ifdef DEBUG_COMMUNICATION
        DEBUG_PRINT("Network Topology: %d:", topology.size);
        for (int i = 0; i < topology.size; i++){
            DEBUG_PRINT("%d ", topology.devices_ids[i]);
        }
        DEBUG_PRINT("\n");
        DEBUG_PRINT("My ID: %d\n", my_id);
    #endif

    return dtrGetSelfId();
}

void p2pcallBackHandler(P2PPacket *p){
	// If the packet is a DTR service packet, then the handler will handle it.
	// It returns true if the packet was handled.

    if (!dtrP2PIncomingHandler(p)){
		// If packet was not handled from DTR , then it is a normal packet 
		// that user has to handle. 

	}
}

bool broadcastState(uint8_t self_id, const state_t *state) {
    dtrPacket statePacket;
    statePacket.dataSize = data_size;
    statePacket.packetSize = DTR_PACKET_HEADER_SIZE + data_size;
    statePacket.sourceId = self_id;
    statePacket.targetId  = 0xFF;
    statePacket.messageType = DATA_FRAME;
    
    #ifdef ENABLE_NEIGHBOR_REL_VEL
        float prep_buffer[6];
        prep_buffer[0] = state->position.x;
        prep_buffer[1] = state->position.y;
        prep_buffer[2] = state->position.z;
        
        prep_buffer[3] = state->velocity.x;
        prep_buffer[4] = state->velocity.y;
        prep_buffer[5] = state->velocity.z;

        // Split the float representation into 4 indexes of bytes.
        // Packet size is number of bytes. uint8_t is one byte.
        memcpy(statePacket.data, prep_buffer, data_size);
    #else
        float prep_buffer[3];
        prep_buffer[0] = state->position.x;
        prep_buffer[1] = state->position.y;
        prep_buffer[2] = state->position.z;

        // Split the float representation into 4 indexes of bytes.
        memcpy(send_buffer, prep_buffer, sizeof(data_size));
    #endif

    bool msg_status;

    msg_status = dtrSendPacket(&statePacket);

    return msg_status;
}

void updateNeighborData(uint8_t my_id, float *rel_distance, const state_t *state, float *neighbor_array) {
    DEBUG_PRINT("Receiving Message...");
    dtrPacket received_packet;

    dtrGetPacket(&received_packet, M2T(10));
    uint8_t other_id = received_packet.sourceId;

    #ifdef ENABLE_NEIGHBOR_REL_VEL
        float other_pos[6];
    #else
        float other_pos[3];
    #endif
    memcpy(other_pos, received_packet.data, data_size);
    float my_pos[3] = {state->position.x, state->position.y, state->position.z};
    float dist = get_dist(other_pos, my_pos);
    // float pos1 = (float) received_packet.data[0];
    // float pos2 = (float) received_packet.data[1];
    // float pos3 = (float) received_packet.data[2];
    #ifdef DEBUG_COMMUNICATION
        DEBUG_PRINT("Received ID=%d: (%f, %f, %f)\n", other_id, other_pos[0], other_pos[1], other_pos[2]);
    #endif
    // Update distances array
    rel_distance[other_id] = dist;

    //Check to see if the robots are closer to yourself.
    for (int i=0; i < NEIGHBOR_ATTENTION; i++) {
        float current_neighbor_pos[3] = {neighbor_array[0], neighbor_array[1], neighbor_array[2]};
        if (dist < get_dist(my_pos, current_neighbor_pos)) {
            #ifdef ENABLE_NEIGHBOR_REL_VEL
                for (int j=0;j<6;j++) {
                    neighbor_array[j+(i*6)] = other_pos[j];
                }
            #else
                for (int j=0;j<3;j++) {
                    neighbor_array[j+(i*3)] = other_pos[j];
                }
            #endif

            //Don't bother checking the other ones
            return;
        }
    }
}   

// Sqrt is an iterative operation. The distance returned is not a true "distance" but just a measure.
float get_dist(float *p1, float *p2) {
    float dx = p1[0] - p2[0];
    float dy = p1[1] - p2[1];
    float dz = p1[2] - p2[2];

    return dx*dx + dy*dy + dz*dz;
}