#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"

#include "log.h"
#include "param.h"

#include "task.h"
#include "math3d.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TOF_NN_CONTROLLER"

// When defined, enables the ToF module.
#define TOF_ENABLE 
// #define DEBUG_LOCALIZATION

// Crazyflie Incldues
#include "debug.h"
#include "controller.h"
#include "stabilizer_types.h"

// Custom Includes
#include "vl53l5cx_api.h"
#include "network_evaluate_tof.h"
#include "network_evaluate_takeoff.h"
#include "I2C_expander.h"

// #define MAX_THRUST 0.15f
// PWM to thrust coefficients
#define A 2.130295e-11f
#define B 1.032633e-6f
#define C 5.484560e-4f

// static float maxThrustFactor = 0.70f;
static bool relVel = false;
static bool relOmega = false;
static bool relXYZ = false;

static control_t_n control_nn;

static VL53L5CX_Configuration f_dev;
static VL53L5CX_ResultsData ranging_data;
static uint8_t sensor_status;

static const float OBST_MAX = 2.0f;
static const float DANGER_DIST = 0.5f;
static const float SAFE_HEIGHT = 0.25f;
static int state_dim = 18;
static int count = 0;

static float state_array[18];
static float setpoint_array[3];

/**
 * @brief Defines the minimum distance of each input column of the ToF sensor. 
 * This is an intermeddiate that is a copy of the sensor matrix.
 * 
 */
static uint16_t tof_input[OBST_DIM*OBST_DIM];

/**
 * @brief Defines the number of valid targets in each zone
 * 
 */
static uint8_t tof_target[OBST_DIM*OBST_DIM];

/**
 * @brief Defines the status of each zone where 5 and 9 means that the range status
 * is OK.
 */
static uint8_t tof_status[OBST_DIM*OBST_DIM];

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
 * @brief Normalize thrusts from 0 to 1 on an unsigned 16 bit integer resolution. 
 */
void normalizeThrust(control_t_n *control_nn, int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3) {
		// scaling and cliping
    // Regular Crazyflie => output thrust directly

    // motor 0
    *PWM_0 = 1.0f * UINT16_MAX * clip(scale(control_nn->thrust_0), 0.0, 1.0);
    // motor 1
    *PWM_1 = 1.0f * UINT16_MAX * clip(scale(control_nn->thrust_1), 0.0, 1.0);
    // motor
    *PWM_2 = 1.0f * UINT16_MAX * clip(scale(control_nn->thrust_2), 0.0, 1.0);
    // motor 3 
    *PWM_3 = 1.0f * UINT16_MAX * clip(scale(control_nn->thrust_3), 0.0, 1.0);
}

/**
 * @brief Collects ToF matrix data. 
 */
void tof_task(VL53L5CX_Configuration* f_dev, VL53L5CX_ResultsData* ranging_data, uint8_t* sensor_status, 
				uint16_t tof_input[OBST_DIM*OBST_DIM], uint8_t tof_target[OBST_DIM*OBST_DIM], uint8_t tof_status[OBST_DIM*OBST_DIM]) {

	vl53l5cx_check_data_ready(f_dev, sensor_status);

	if (*sensor_status){
		vl53l5cx_get_ranging_data(f_dev, ranging_data);
		memcpy(tof_input, (uint16_t *)(&ranging_data->distance_mm[0]), OBST_DIM*OBST_DIM*2);
		// memcpy(tof_target, (uint8_t *)(&ranging_data->nb_target_detected[0]), OBST_DIM*OBST_DIM);
    	memcpy(tof_status, (uint8_t *)(&ranging_data->target_status[0]), OBST_DIM*OBST_DIM);
	}
}

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {

	//Initialize sensor platform
	#ifdef TOF_ENABLE
	f_dev.platform = VL53L5CX_DEFAULT_I2C_ADDRESS;
	DEBUG_PRINT("Set VL53L5CX Address\n"); 

	vTaskDelay(M2T(100)); 

	sensor_status = vl53l5cx_init(&f_dev);
	if (sensor_status == 0) {
		DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
	}
	#ifdef ENABLE_4X4_CONTROLLER
		// Sets the sensor to be 8x8.
		sensor_status = vl53l5cx_set_resolution(&f_dev, VL53L5CX_RESOLUTION_4X4);
		if (sensor_status == 0) {
			DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
		}
		sensor_status = vl53l5cx_set_ranging_frequency_hz(&f_dev, 30);

		if (sensor_status == 0) {
			DEBUG_PRINT("VL53L5CX Frequency Config: Pass\n"); 
		} 
	#else
		// Sets the sensor to be 8x8.
		sensor_status = vl53l5cx_set_resolution(&f_dev, VL53L5CX_RESOLUTION_8X8);
		if (sensor_status == 0) {
			DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
		}
		sensor_status = vl53l5cx_set_ranging_frequency_hz(&f_dev, 15);

		if (sensor_status == 0) {
			DEBUG_PRINT("VL53L5CX Frequency Config: Pass\n"); 
		} 
	#endif
	//Below function should be the last called in the init. 
	sensor_status = vl53l5cx_start_ranging(&f_dev);
	if (sensor_status == 0) {
		DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
	} 

	vTaskDelay(M2T(100));
	#endif

	while(1) {
		#ifdef ENABLE_4X4_CONTROLLER
			vTaskDelay(M2T(34)); // 30Hz is roughly 30 ms intervals
		#else
			vTaskDelay(M2T(67)); // 15Hz is roughly 67 ms intervals
		#endif
		tof_task(&f_dev, &ranging_data, &sensor_status, tof_input, tof_target, tof_status);
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
	// Initialize starting obs as max value
	for (int i=0;i<OBST_DIM;i++) {
		obstacle_inputs[i] = 2.0f;
	}

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
			DEBUG_PRINT("Estimation: (%f,%f,%f)\n", state->position.x,state->position.y,state->position.z);
			DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
			DEBUG_PRINT("ERROR: (%f,%f,%f)\n", state_array[0], state_array[1], state_array[2]);
			count = 0;
		}
		count++;
	#endif

	if (relVel) {
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

	if (relXYZ) {
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

	if (relOmega) {
		state_array[15] = omega_roll - radians(setpoint->attitudeRate.roll);
		state_array[16] = omega_pitch - radians(setpoint->attitudeRate.pitch);
		state_array[17] = omega_yaw - radians(setpoint->attitudeRate.yaw);
	} else {
		state_array[15] = omega_roll;
		state_array[16] = omega_pitch;
		state_array[17] = omega_yaw;
	}

	if (state_dim == 19) {
		state_array[18] = state->position.z;
	}
	
	//Implementation One: Use only the values of a specific column
	// DEBUG_PRINT("TOF Controller Value: %i\n", tof_input[39]);
	// NOTE: The ToF lens flips the image plane vertically and horizontally
	#ifdef ENABLE_4X4_CONTROLLER
		// ToF Measurements are noisy on takeoff. 
		if (state->position.z > SAFE_HEIGHT) {
			int row_index = 4;
			// NOTE: THIS ONLY ITERATES THROUGH THE FIRST 4 OBSERVATIONS
			for (int i=0;i<4;i++) {
				int curr_index = row_index + i;
				// Check if the pixels are valid 
				if ((tof_status[curr_index] == 9) || (tof_status[curr_index] == 5)) {
					float obst_cap;
					obst_cap = tof_input[curr_index];
					obst_cap = obst_cap / 1000.0f;

					if ((obst_cap > OBST_MAX)) {
						obst_cap = OBST_MAX;
					}
					obstacle_inputs[i] = obst_cap;
					// obstacle_inputs[i] = OBST_MAX; //Ablate inputs to NN
				} else {
					// DEBUG_PRINT("Invalid Reading!");
					obstacle_inputs[i] = OBST_MAX;
				}
			}
		} else {
			for (int i=0;i<OBST_DIM;i++) {
				obstacle_inputs[i] = OBST_MAX;
			}
		}

	#else
	
	if (state->position.z > SAFE_HEIGHT) {
		int row_index = 32;
		for (int i=0;i<8;i++) {
			int curr_index = row_index + i;
			if ((tof_status[curr_index] == 9) || (tof_status[curr_index] == 5)) {
				float obst_cap;
				obst_cap = tof_input[curr_index];
				obst_cap = obst_cap / 1000.0f;

				if ((obst_cap > OBST_MAX)) {
					obst_cap = OBST_MAX;
				}
				obstacle_inputs[i] = obst_cap;
				// obstacle_inputs[i] = OBST_MAX;
			} else {
				// DEBUG_PRINT("Invalid Reading!");
				obstacle_inputs[i] = OBST_MAX;
			}
			// obstacle_inputs[i] = 2.0f;
			// tof_input[i] = 2.0f;
		}	

		// for (int i=0;i<OBST_DIM;i++) {
		// 	obstacle_inputs[i] = OBST_MAX;
		// } 
	} else {
		for (int i=0;i<OBST_DIM;i++) {
			obstacle_inputs[i] = OBST_MAX;
		}
	}
	#endif

	obstacleEmbedder(obstacle_inputs);
	// networkEvaluate(&control_nn, state_array);
	networkEvaluateTakeoff(&control_nn, state_array);

	// convert thrusts to normalized Thrust
	int iThrust_0, iThrust_1, iThrust_2, iThrust_3; 
	normalizeThrust(&control_nn, &iThrust_0, &iThrust_1, &iThrust_2, &iThrust_3);

	if (setpoint->mode.z == modeDisable) {
		control->normalizedForces[0] = 0;
		control->normalizedForces[1] = 0;
		control->normalizedForces[2] = 0;
		control->normalizedForces[3] = 0;
	} else {
		control->normalizedForces[0] = iThrust_0;
		control->normalizedForces[1] = iThrust_1;
		control->normalizedForces[2] = iThrust_2;
		control->normalizedForces[3] = iThrust_3;
	}
}

LOG_GROUP_START(ctrlNN)

LOG_ADD(LOG_FLOAT, set_x, &setpoint_array[0])
LOG_ADD(LOG_FLOAT, set_y, &setpoint_array[1])
LOG_ADD(LOG_FLOAT, set_z, &setpoint_array[2])

LOG_GROUP_STOP(ctrlNN)