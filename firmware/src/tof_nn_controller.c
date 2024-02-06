#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "math3d.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TOF_NN_CONTROLLER"

// Crazyflie Incldues
#include "debug.h"
#include "controller.h"
#include "stabilizer_types.h"
#include "app.h"

// Custom Includes
#include "vl53l5cx_api.h"
#include "network_evaluate_tof.h"

#define MAX_THRUST 0.15f
// PWM to thrust coefficients
#define A 2.130295e-11f
#define B 1.032633e-6f
#define C 5.484560e-4f

// static float maxThrustFactor = 0.70f; Is not used.
static bool relVel = true;
static bool relOmega = true;
static bool relXYZ = true;
static uint16_t freq = 500;

static control_t_n control_nn;
static struct mat33 rot;
static uint32_t usec_eval;

static VL53L5CX_Configuration front_sensor;
static VL53L5CX_ResultsData ranging_data;
static uint8_t sensor_status;

/**
 * @brief Defines the input array for the neural network. First 18 are state variables of the quadcopter. 
 * 
 */
static float state_array[18];

/**
 * @brief Defines the minimum distance of each input column of the ToF sensor. 
 * 
 */
static float tof_input[OBST_DIM];

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
 * @brief Collects and processes ToF matrix data. 
 * Transforms the data into the flattened vector that the network accepts.
 * 
 * @param p_dev 
 * @param p_results
 * @param tof_input
 */
void concatTOF(VL53L5CX_Configuration *p_dev, VL53L5CX_ResultsData *p_data, float (*tof_input)[]) {
	vl53l5cx_check_data_ready(p_dev, &sensor_status);

	if (sensor_status){
		vl53l5cx_get_ranging_data(p_dev, p_data);
		//Implementation One: Use ONLY the middle of each column
		if(1) {
			for (int i=0;i<8;i++) {
				(*tof_input)[i] = (p_data->distance_mm[31-i] + p_data->distance_mm[39-i])/2;
			}
		}
	}
}

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void controllerOutOfTreeInit() {
	//Initialize all control values to 0.
	control_nn.thrust_0 = 0.0f;
	control_nn.thrust_1 = 0.0f;
	control_nn.thrust_2 = 0.0f;
	control_nn.thrust_3 = 0.0f;

	uint8_t alive_status = vl53l5cx_is_alive(&front_sensor, &sensor_status);

	if(!sensor_status || alive_status) {
		DEBUG_PRINT("VL53L5CX Is Not Connected!");
	}
	//Initialize sensor platform
	front_sensor.platform = VL53L5CX_DEFAULT_I2C_ADDRESS;
	uint8_t init_status = vl53l5cx_init(&front_sensor);
	DEBUG_PRINT("VL53L5CX Initialize: %s\n", init_status ? "PASS." : "FAIL");  
	// Sets the sensor to be 8x8.
	uint8_t res_status = vl53l5cx_set_resolution(&front_sensor, VL53L5CX_RESOLUTION_8X8);
	DEBUG_PRINT("VL53L5CX Set Resolution: %s\n", res_status ? "PASS." : "FAIL");  
	//Below function should be the last called in the init. 
	uint8_t ranging_status = vl53l5cx_start_ranging(&front_sensor);
	DEBUG_PRINT("VL53L5CX Start Ranging: %s\n", ranging_status ? "PASS." : "FAIL");  
}
bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  control->controlMode =  controlModeForce;
	if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/freq, tick)) {
		return;
	}

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

	state_array[0] = state->position.x - setpoint->position.x;
	state_array[1] = state->position.y - setpoint->position.y;
	state_array[2] = state->position.z - setpoint->position.z;
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
	concatTOF(&front_sensor, &ranging_data, &tof_input);
	obstacleEmbedder(tof_input);
	// run the neural neural network
	uint64_t start = usecTimestamp();
	networkEvaluate(&control_nn, state_array);
	usec_eval = (uint32_t) (usecTimestamp() - start);

	// convert thrusts to normalized Thrust
	// need to hack the firmware (stablizer.c and power_distribution_stock.c)
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