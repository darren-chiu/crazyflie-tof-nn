/**
 * @file tof_nn_controller.c
 * @author Darren Chiu (chiudarr@usc.edu)
 * @brief 
 * @version 2.0
 * @date 2024-04-25
 * 
 * @copyright Copyright (c) 2024
 * 
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"

#include "log.h"
#include "param.h"

#include "task.h"
#include "timers.h"
#include "math3d.h"


// Crazyflie Incldues
#include "debug.h" 
#include "controller.h"
#include "stabilizer_types.h"
#include "obst_daq.h"
#include "p2p_state.h"
#include "network_evaluate_tof.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TOF_NN_CONTROLLER"

// Output from the neural network.
static control_t_n control_nn;

// Observations
static uint8_t sensor_status;
static VL53L5CX_Configuration tof_config;
static uint16_t tof_addresses[NUM_SENSORS] = {0x50, 0x66, 0x76, 0x86};

// Bool that tracks when the obstacle embedder needs to be updated.
static bool isToFStale = false;

//This would be the input to the network
static float neighbor_inputs[NEIGHBORS*NBR_OBS_DIM];

// Timer that tracks peer to peer exchange. 
static xTimerHandle P2PPosTimer;
// Timer that tracks the observation request process. 
static xTimerHandle ObservationTimer;

// Dynamics Parameters
static float state_array[STATE_DIM];
static float setpoint_array[6];


static int count = 0;
static float thrust_coefficient = 1.0;
static uint16_t thrust_offset = 500;


/**
 * @brief Defines the minimum distance of each input column of the ToF sensor. 
 * This is an intermeddiate that is a copy of the sensor matrix.
 * 
 */
static uint16_t tof_input[NUM_SENSORS * OBST_DIM];

/**
 * @brief Defines the status of each zone where 5 and 9 means that the range status
 * is OK.
 */
static uint8_t tof_status[NUM_SENSORS * OBST_DIM];

/**
 * @brief The input vector seen for the obstacle encoder. 
 * 
 */
static float obstacle_inputs[OBST_DIM];

/**
 * @brief Scale the given V to a range from 0 to 1.
 * 
 * @param v 
 * @return float 
 */
float scale(float v) {
	return 0.5f * (v + 1);
}


/**
 * @brief Clip the input V if out of bounds from (min, max)
 * 
 * @param v 
 * @param min 
 * @param max 
 * @return float 
 */
float clip(float v, float min, float max) {
	if (v < min) return min;
	if (v > max) return max;
	return v;
}

/**
 * @brief Used by RTOS to match sensor timing
 * 
 * @param timer 
 */
static void pullObs(xTimerHandle timer) {
	isToFStale = tof_task(&tof_config, tof_addresses, &sensor_status, tof_input, tof_status);
}

/**
 * @brief Normalize thrusts from 0 to 1 on an unsigned 16 bit integer resolution. 
 */
void normalizeThrust(control_t_n *control_nn, uint16_t *PWM_0, uint16_t *PWM_1, uint16_t *PWM_2, uint16_t *PWM_3) {
		// scaling and cliping
    // Regular Crazyflie => output thrust directly

    // motor 0
    *PWM_0 = UINT16_MAX * clip(scale(control_nn->thrust_0), 0.0, 1.0);
    // motor 1
    *PWM_1 = UINT16_MAX * clip(scale(control_nn->thrust_1), 0.0, 1.0);
    // motor
    *PWM_2 = UINT16_MAX * clip(scale(control_nn->thrust_2), 0.0, 1.0);
    // motor 3 
    *PWM_3 = UINT16_MAX * clip(scale(control_nn->thrust_3), 0.0, 1.0);
}

static void sendData(xTimerHandle timer) {
	//Only Start sending messages when we are above the safe height. 
	message_state_t tx_message;

	
	tx_message.x_pos = getX();
	tx_message.y_pos = getY();
	tx_message.z_pos = getZ();

	#ifdef ENABLE_NEIGHBOR_REL_VEL
		tx_message.x_vel = getVx();
		tx_message.y_vel = getVy();
		tx_message.z_vel = getVz();
	#endif

	broadcastState(tx_message);
}

void appMain() {	

	//Initialize sensor platform
	#ifdef TOF_ENABLE
		//Initialize sensor platform
		tof_init(&tof_config, tof_addresses);
		
		vTaskDelay(M2T(10));
		//Start RTOS task for observations. The task will thus run at M2T(67) ~ 15Hz.
		ObservationTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
		xTimerStart(ObservationTimer, 20);
	#endif

	//Start RTOS task for broadcasting
	#ifdef MULTI_DRONE_ENABLE
		// Network Initialization
		network_init();
		P2PPosTimer = xTimerCreate("P2PPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, sendData);
		xTimerStart(P2PPosTimer, 20);
	#endif



	while(1) {
		vTaskDelay(M2T(500));
		#ifdef DEBUG_LOCALIZATION
			// DEBUG_PRINT("Estimation: (%f,%f,%f)\n", state->position.x,state->position.y,state->position.z);
			// DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
			// DEBUG_PRINT("ERROR: (%f,%f,%f)\n", state_array[0], state_array[1], state_array[2]);
			DEBUG_PRINT("ToF: (%f,%f,%f,%f,%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);
			// DEBUG_PRINT("ToF: (%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[3], obstacle_inputs[5], obstacle_inputs[7]);
			// DEBUG_PRINT("Thrusts: (%i,%i,%i,%i)\n", control->normalizedForces[0], control->normalizedForces[1], control->normalizedForces[2], control->normalizedForces[3]);
		#endif
	}
}

void controllerOutOfTreeInit() {
	//Initialize all control values to 0.
	control_nn.thrust_0 = 0.0f;
	control_nn.thrust_1 = 0.0f;
	control_nn.thrust_2 = 0.0f;
	control_nn.thrust_3 = 0.0f;

}
bool controllerOutOfTreeTest() {
  	// Always return true

	return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
	control->controlMode = controlModeForce;
	bool comm_status;

	struct mat33 rot;
	struct mat33 rot_desired;

	// Orientation
	struct quat q = mkquat(state->attitudeQuaternion.x, 	
						   state->attitudeQuaternion.y, 
						   state->attitudeQuaternion.z, 
						   state->attitudeQuaternion.w);
	rot = quat2rotmat(q);

	struct quat q_desired = mkquat(setpoint->attitudeQuaternion.x, 
						   setpoint->attitudeQuaternion.y, 
						   setpoint->attitudeQuaternion.z, 
						   setpoint->attitudeQuaternion.w);
	rot_desired = quat2rotmat(q_desired);


	// angular velocity
	float omega_roll = radians(sensors->gyro.x);
	float omega_pitch = radians(sensors->gyro.y);
	float omega_yaw = radians(sensors->gyro.z);

	setpoint_array[0] = setpoint->position.x;
	setpoint_array[1] = setpoint->position.y;
	setpoint_array[2] = setpoint->position.z;

	state_array[0] = state->position.x - setpoint->position.x;
	state_array[1] = state->position.y - setpoint->position.y;
	state_array[2] = state->position.z - setpoint->position.z;

	if (REL_VEL) {
		state_array[3] = state->velocity.x - setpoint->velocity.x;
		state_array[4] = state->velocity.y - setpoint->velocity.y;
		state_array[5] = state->velocity.z - setpoint->velocity.z;
	} else {
		state_array[3] = state->velocity.x;
		state_array[4] = state->velocity.y;
		state_array[5] = state->velocity.z;
	}
	
	if (REL_ROT) {
		state_array[6] = rot.m[0][0] - rot_desired.m[0][0];
		state_array[7] = rot.m[0][1] - rot_desired.m[0][1];
		state_array[8] = rot.m[0][2] - rot_desired.m[0][2];
		state_array[9] = rot.m[1][0] - rot_desired.m[1][0];
		state_array[10] = rot.m[1][1] - rot_desired.m[1][1];
		state_array[11] = rot.m[1][2] - rot_desired.m[1][2];
		state_array[12] = rot.m[2][0] - rot_desired.m[2][0];
		state_array[13] = rot.m[2][1] - rot_desired.m[2][1];
		state_array[14] = rot.m[2][2] - rot_desired.m[2][2];
	} else {
		state_array[6] = rot.m[0][0];
		state_array[7] = rot.m[0][1];
		state_array[8] = rot.m[0][2];
		state_array[9] = rot.m[1][0];
		state_array[10] = rot.m[1][1];
		state_array[11] = rot.m[1][2];
		state_array[12] = rot.m[2][0];
		state_array[13] = rot.m[2][1];
		state_array[14] = rot.m[2][2];
	}

	if (REL_XYZ) {
		// rotate pos and vel
		struct vec rot_pos = mvmul(mtranspose(rot), mkvec(state_array[0], state_array[1], state_array[2]));
		struct vec rot_vel = mvmul(mtranspose(rot), mkvec(state_array[3], state_array[4], state_array[5]));

		state_array[0] = rot_pos.x;
		state_array[1] = rot_pos.y;
		state_array[2] = rot_pos.z;

		state_array[3] = rot_vel.x;
		state_array[4] = rot_vel.y;
		state_array[5] = rot_vel.z;
	}

	setpoint_array[3] = radians(setpoint->attitudeRate.roll);
	setpoint_array[4] = radians(setpoint->attitudeRate.pitch);
	setpoint_array[5] = radians(setpoint->attitudeRate.yaw);

	if (REL_OMEGA) {
		state_array[15] = omega_roll - setpoint_array[4];
		state_array[16] = omega_pitch - setpoint_array[5];
		state_array[17] = omega_yaw - setpoint_array[6];
	} else {
		state_array[15] = omega_roll;
		state_array[16] = omega_pitch;
		state_array[17] = omega_yaw;
	}

	if (!isToFStale) {
		#ifdef TOF_ENABLE
			isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
			obstacleEmbedder(obstacle_inputs);
		#endif
	}

	#ifdef MULTI_DRONE_ENABLE
		updateNeighborInputs(state, neighbor_inputs);
		neighborEmbedder(neighbor_inputs);  
	#endif


	networkEvaluate(&control_nn, state_array);

	// convert thrusts to normalized Thrust
	uint16_t iThrust_0, iThrust_1, iThrust_2, iThrust_3; 
	normalizeThrust(&control_nn, &iThrust_0, &iThrust_1, &iThrust_2, &iThrust_3);

	if (setpoint->mode.z == modeDisable) {
		control->normalizedForces[0] = 0;
		control->normalizedForces[1] = 0;
		control->normalizedForces[2] = 0;
		control->normalizedForces[3] = 0;
	} else {
		control->normalizedForces[0] = (uint16_t) (thrust_coefficient * iThrust_0 + thrust_offset);
		control->normalizedForces[1] = (uint16_t) (thrust_coefficient * iThrust_1 + thrust_offset);
		control->normalizedForces[2] = (uint16_t) (thrust_coefficient * iThrust_2 + thrust_offset);
		control->normalizedForces[3] = (uint16_t) (thrust_coefficient * iThrust_3 + thrust_offset);
	}
}

/**
 * [Documentation for the ring group ...]
 */
PARAM_GROUP_START(paramNN)
/**
 * @brief to start the flight
 */
PARAM_ADD_CORE(PARAM_UINT16, thrust_offset, &thrust_offset)
PARAM_GROUP_STOP(paramNN)

LOG_GROUP_START(logNN)

LOG_ADD(LOG_FLOAT, set_x, &setpoint_array[0])
LOG_ADD(LOG_FLOAT, set_y, &setpoint_array[1])
LOG_ADD(LOG_FLOAT, set_z, &setpoint_array[2])


LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])


LOG_GROUP_STOP(logNN)