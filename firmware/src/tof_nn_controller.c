#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"

#include "log.h"
#include "param.h"

#include "task.h"
#include "math3d.h"


// Crazyflie Incldues
#include "debug.h"
#include "controller.h"
#include "stabilizer_types.h"
#include "obst_daq.h"

#include "network_evaluate_tof.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TOF_NN_CONTROLLER"

// Output from the neural network.
static control_t_n control_nn;

static uint8_t sensor_status;
static VL53L5CX_Configuration tof_array[NUM_SENSORS];
static VL53L5CX_ResultsData ranging_data[NUM_SENSORS];
static uint16_t tof_addresses[NUM_SENSORS] = {0x40, 0x44, 0x48, 0x50};

// Bool that tracks when the obstacle embedder needs to be updated.
static bool isStale = false;

// Dynamics Parameters
static float state_array[STATE_DIM];
static float setpoint_array[6];

// Debugging Parameters
#ifdef DEBUG_LOCALIZATION
	static int count = 0;
#endif
static float thrust_coefficient = 0;


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
static float obstacle_inputs[OBST_DIM] = {2.0f};

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

void appMain() {
	//Initialize sensor platform
	#ifdef TOF_ENABLE
	tof_init(&sensor_status, tof_array, tof_addresses);

	vTaskDelay(M2T(100));
	#endif

	while(1) {
		#ifdef ENABLE_4X4_CONTROLLER
			vTaskDelay(M2T(34)); // 30Hz is roughly 30 ms intervals
		#else
			vTaskDelay(M2T(67)); // 15Hz is roughly 67 ms intervals
		#endif
		isStale = tof_task(tof_array, ranging_data, &sensor_status, (uint16_t* )tof_input, (uint8_t* ) tof_status);
	}
}

void controllerOutOfTreeInit() {
	//Initialize all control values to 0.
	control_nn.thrust_0 = 0.0f;
	control_nn.thrust_1 = 0.0f;
	control_nn.thrust_2 = 0.0f;
	control_nn.thrust_3 = 0.0f;

	// uint8_t alive_status
	// sensor_status = vl53l5cx_is_alive(&f_dev, &sensor_status);

	// if(!sensor_status || alive_status) {
	// 	DEBUG_PRINT("VL53L5CX Is Alive!");
	// }
}
bool controllerOutOfTreeTest() {
  	// Always return true

	return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
	control->controlMode = controlModeForce;	

	struct mat33 rot;

	// Orientation
	struct quat q = mkquat(state->attitudeQuaternion.x, 
						   state->attitudeQuaternion.y, 
						   state->attitudeQuaternion.z, 
						   state->attitudeQuaternion.w);
	rot = quat2rotmat(q);

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

	#ifdef DEBUG_LOCALIZATION
		if (count == 500) {
			DEBUG_PRINT("Estimation: (%f,%f,%f,%f)\n", state->position.x,state->position.y,state->position.z, thrust_coefficient);
			// DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
			// DEBUG_PRINT("ERROR: (%f,%f,%f)\n", state_array[0], state_array[1], state_array[2]);
			DEBUG_PRINT("ToF: (%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3]);
			// DEBUG_PRINT("Thrusts: (%i,%i,%i,%i)\n", control->normalizedForces[0], control->normalizedForces[1], control->normalizedForces[2], control->normalizedForces[3]);
			count = 0;
		}
		count++;
	#endif

	if (REL_VEL) {
		state_array[3] = state->velocity.x - setpoint->velocity.x;
		state_array[4] = state->velocity.y - setpoint->velocity.y;
		state_array[5] = state->velocity.z - setpoint->velocity.z;
	} else {
		state_array[3] = state->velocity.x;
		state_array[4] = state->velocity.y;
		state_array[5] = state->velocity.z;
	}
	state_array[6] = rot.m[0][0];
	state_array[7] = rot.m[0][1];
	state_array[8] = rot.m[0][2];
	state_array[9] = rot.m[1][0];
	state_array[10] = rot.m[1][1];
	state_array[11] = rot.m[1][2];
	state_array[12] = rot.m[2][0];
	state_array[13] = rot.m[2][1];
	state_array[14] = rot.m[2][2];

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

	if (!isStale) {
		isStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
		obstacleEmbedder(obstacle_inputs);
	}
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
		control->normalizedForces[0] = thrust_coefficient * iThrust_0;
		control->normalizedForces[1] = thrust_coefficient * iThrust_1;
		control->normalizedForces[2] = thrust_coefficient * iThrust_2;
		control->normalizedForces[3] = thrust_coefficient * iThrust_3;
	}
}

LOG_GROUP_START(ctrlNN)

LOG_ADD(LOG_FLOAT, set_x, &setpoint_array[0])
LOG_ADD(LOG_FLOAT, set_y, &setpoint_array[1])
LOG_ADD(LOG_FLOAT, set_z, &setpoint_array[2])


LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])


LOG_GROUP_STOP(ctrlNN)