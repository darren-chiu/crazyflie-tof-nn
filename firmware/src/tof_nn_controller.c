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
#include "network_config.h"
#include "platform_defaults.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TOF_NN_CONTROLLER"

float omega_roll;
float omega_pitch; 
float omega_yaw;

// Output from the neural network.
static control_t_n control_nn;
static struct vec control_torque;

// Observations
static uint8_t sensor_status;
static VL53L5CX_Configuration tof_config;

#define UPDATE_RATE RATE_500_HZ

static uint16_t iThrust_0, iThrust_1, iThrust_2, iThrust_3; 
static float iThrustF_0, iThrustF_1, iThrustF_2, iThrustF_3; 

// Max thrust of a single rotor
static float single_rotor_max_thrust = 0.28395352f;

// Pulled from the simulation. Includes ToF deck. 
static struct mat33 CRAZYFLIE_INERTIA =
    {{{2.6214e-5f, 0.0, 0.0},
      {0.0, 2.5611e-5f, 0.0},
      {0.0, 0.0, 4.40571e-5f}}};

// The mass that includes the new ToF Deck
static float CRAZYFLIE_MASS = 0.048751f;

// Dynamics Parameters
static float state_array[STATE_DIM];
static float setpoint_array[9];

static float thrust_coefficient = 1.0;
static uint16_t thrust_offset = 600;

// time constant of rotational rate control
static float tau_rp_rate = 0.015; // 0.015
static float tau_yaw_rate = 0.75;

// minimum and maximum body rates
static float omega_rp_max = 7.5;
static float omega_yaw_max = 2.5;

static float armLength = 0.022f; // m
static float thrustToTorque = 0.005964552f;

float lin_transform(float a, float out_min, float out_max) {
	// Linear transform value a into range [out_max, out_min] given input ranges [in_max, in_min]
	float in_max = 1.0;
	float in_min = 0.0;

	return (a - in_min) * ((out_max - out_min)/(in_max - in_min)) + out_min;
}

#ifdef MULTI_DRONE_ENABLE
	// Timer that tracks peer to peer exchange. 
	static xTimerHandle P2PPosTimer;

	//This would be the input to the network. Volatile to get rid of compiler optimizations.
	static volatile float neighbor_inputs[NEIGHBORS*NBR_OBS_DIM];

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

#endif

#ifdef TOF_ENABLE
	// Timer that tracks the observation request process. 
	static xTimerHandle ObservationTimer;

	// Bool that tracks when the obstacle embedder needs to be updated.
	static bool isToFStale = false;

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
	static volatile float obstacle_inputs[OBST_DIM];

	/**
	 * @brief Used by RTOS to match sensor timing
	 * 
	 * @param timer 
	*/
	static void pullObs(xTimerHandle timer) {
		isToFStale = tof_task(&tof_config, tof_addresses, &sensor_status, tof_input, tof_status);
	}


#endif

/**
 * @brief Scale the given V to a range from 0 to 1.
 */
float scale(float v) {
	return 0.5f * (v + 1.0);
}


/**
 * @brief Clip the input V if out of bounds from (min, max)
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
    // motor 0
    *PWM_0 = UINT16_MAX * clip(scale(control_nn->thrust_0), -1.0, 1.0);
    // motor 1
    *PWM_1 = UINT16_MAX * clip(scale(control_nn->thrust_1), -1.0, 1.0);
    // motor
    *PWM_2 = UINT16_MAX * clip(scale(control_nn->thrust_2), -1.0, 1.0);
    // motor 3 
    *PWM_3 = UINT16_MAX * clip(scale(control_nn->thrust_3), -1.0, 1.0);
}
/**
 * @brief Normalize thrusts from 0 to 1 but returns a float representation.
 */
void normalizeThrustFloat(control_t_n *control_nn, float *iThrustF_0, float *iThrustF_1, float *iThrustF_2, float *iThrustF_3) {
    // motor 0
    *iThrustF_0 = scale(clip(control_nn->thrust_0, -1.0, 1.0));
    // motor 1
    *iThrustF_1 = scale(clip(control_nn->thrust_1, -1.0, 1.0));
    // motor
    *iThrustF_2 = scale(clip(control_nn->thrust_2, -1.0, 1.0));
    // motor 3 
    *iThrustF_3 = scale(clip(control_nn->thrust_3, -1.0, 1.0));
}

void appMain() {	

	//By default, we should disable all ToFs.
	tof_disable_all();

	initLogIds();

	//Initialize observation sensor platform
	#ifdef TOF_ENABLE
		//Initialize sensor platform
		tof_init(&tof_config, tof_addresses);
		
		vTaskDelay(M2T(10));
		#ifdef ENABLE_4X4_CONTROLLER
			//The task will thus run at M2T(34) ~ 30Hz.
			ObservationTimer = xTimerCreate("ObservationTimer", M2T(34), pdTRUE, NULL, pullObs);
		#else
			//The task will thus run at M2T(67) ~ 15Hz.
			ObservationTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
		#endif
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
		vTaskDelay(M2T(10));
		#ifdef DEBUG_LOCALIZATION
			// DEBUG_PRINT("Estimation: (%f,%f,%f)\n", getX(), getY(), getZ());
			// DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
			// DEBUG_PRINT("ERROR: (%f,%f,%f)\n", -1*state_array[0], -1*state_array[1], -1*state_array[2]);
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

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
	control->controlMode = controlModeForce;

	// angular velocity
	omega_roll = radians(sensors->gyro.x);
	omega_pitch = radians(sensors->gyro.y);
	omega_yaw = radians(sensors->gyro.z);

	// if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/UPDATE_RATE, stabilizerStep)) {
	// 	return;
	// }

	if (RATE_DO_EXECUTE(UPDATE_RATE, stabilizerStep)) {
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


		setpoint_array[0] = setpoint->position.x;
		setpoint_array[1] = setpoint->position.y;
		setpoint_array[2] = setpoint->position.z;
		setpoint_array[3] = radians(setpoint->attitudeRate.roll);
		setpoint_array[4] = radians(setpoint->attitudeRate.pitch);
		setpoint_array[5] = radians(setpoint->attitudeRate.yaw);
		setpoint_array[6] = setpoint->velocity.x;
		setpoint_array[7] = setpoint->velocity.y;
		setpoint_array[8] = setpoint->velocity.z;


		if (REL_XYZ) {
			state_array[0] = state->position.x - setpoint->position.x;
			state_array[1] = state->position.y - setpoint->position.y;
			state_array[2] = state->position.z - setpoint->position.z;
		} else {
			state_array[0] = state->position.x;
			state_array[1] = state->position.y;
			state_array[2] = state->position.z;	
		}

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

		if (REL_OMEGA) {
			state_array[15] = omega_roll - radians(setpoint->attitudeRate.roll);
			state_array[16] = omega_pitch - radians(setpoint->attitudeRate.pitch);
			state_array[17] = omega_yaw - radians(setpoint->attitudeRate.yaw);
		} else {
			state_array[15] = omega_roll;
			state_array[16] = omega_pitch;
			state_array[17] = omega_yaw;
		}

		#ifdef TOF_ENABLE
			if (!isToFStale) {
				isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
				obstacleEmbedder(obstacle_inputs);
			}
		#endif

		#ifdef MULTI_DRONE_ENABLE
			updateNeighborInputs(state, neighbor_inputs);
			neighborEmbedder(neighbor_inputs);  
			singleHeadAttention();
		#endif

		networkEvaluate(&control_nn, state_array);	

	} else {
		if (USE_CTBR) {
			if (setpoint->mode.z == modeDisable) {
				control->normalizedForces[0] = 0;
				control->normalizedForces[1] = 0;
				control->normalizedForces[2] = 0;
				control->normalizedForces[3] = 0;
			} else {
				normalizeThrustFloat(&control_nn, &iThrustF_0, &iThrustF_1, &iThrustF_2, &iThrustF_3);
			
				float control_thrust = lin_transform(iThrustF_0, CRAZYFLIE_MASS, single_rotor_max_thrust*4.0);

				float control_roll = lin_transform(iThrustF_1, -35.0, 35.0);
				float control_pitch = lin_transform(iThrustF_2, -35.0, 35.0);
				float control_yaw = lin_transform(iThrustF_3, -35.0, 35.0);


				// // apply the rotation heuristic
				// if (control_roll * omega_roll < 0 && fabsf(omega_roll) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
				// 	control_roll = omega_rp_max * (omega_roll < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
				// }

				// if (control_pitch * omega_pitch < 0 && fabsf(omega_pitch) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
				// 	control_pitch = omega_rp_max * (omega_pitch < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
				// }

				// if (control_yaw * omega_yaw < 0 && fabsf(omega_yaw) > heuristic_yaw) { // desired rotational rate in direction opposite to current rotational rate
				// 	control_yaw = omega_rp_max * (omega_yaw < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
				// }

				// scale the commands to satisfy rate constraints
				float scaling = 1.0f;
				scaling = fmax(scaling, fabsf(control_roll) / omega_rp_max);
				scaling = fmax(scaling, fabsf(control_pitch) / omega_rp_max);
				scaling = fmax(scaling, fabsf(control_yaw) / omega_yaw_max);

				control_roll /= scaling;
				control_pitch /= scaling;
				control_yaw /= scaling;

				// control the body torques
				struct vec omegaErr = mkvec((control_roll - omega_roll)/tau_rp_rate,
									(control_pitch - omega_pitch)/tau_rp_rate,
									(control_yaw - omega_yaw)/tau_yaw_rate);

				// update the commanded body torques based on the current error in body rates
				control_torque = mvmul(CRAZYFLIE_INERTIA, omegaErr);

				// control->thrustSi = control_thrust; // force to provide control_thrust
				// control->torqueX = control_torque.x;
				// control->torqueY = control_torque.y;
				// control->torqueZ = control_torque.z;


				const float arm = 0.707106781f * armLength;
				const float thrustPart = 0.25f * control_thrust; // N (per rotor)
				const float rollPart = 0.25f / arm * control_torque.x;
				const float pitchPart = 0.25f / arm * control_torque.y;
				const float yawPart = 0.25f * control_torque.z / thrustToTorque;

				float motor_forces_float[4] = {0.0f, 0.0f, 0.0f, 0.0f};

				motor_forces_float[0] = (thrustPart - rollPart - pitchPart - yawPart) / single_rotor_max_thrust;
				motor_forces_float[1] = (thrustPart - rollPart + pitchPart + yawPart) / single_rotor_max_thrust;
				motor_forces_float[2] = (thrustPart + rollPart + pitchPart - yawPart) / single_rotor_max_thrust;
				motor_forces_float[3] = (thrustPart + rollPart - pitchPart + yawPart) / single_rotor_max_thrust;

				for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++) {
					control->normalizedForces[i] = (uint16_t)  (UINT16_MAX * clip(motor_forces_float[i], 0.0, 1.0));
				}


				// DEBUG_PRINT("Control Cmds: (%i,%i,%i,%i)\n", control->normalizedForces[0], control->normalizedForces[1], control->normalizedForces[2], control->normalizedForces[3]);



			}
		} else {
			// convert thrusts to normalized Thrust
			normalizeThrust(&control_nn, &iThrust_0, &iThrust_1, &iThrust_2, &iThrust_3);

			if (setpoint->mode.z == modeDisable) {
				control->normalizedForces[0] = 0;
				control->normalizedForces[1] = 0;
				control->normalizedForces[2] = 0;
				control->normalizedForces[3] = 0;
			} else {
				control->normalizedForces[0] = (uint16_t) (iThrust_0 + thrust_offset);
				control->normalizedForces[1] = (uint16_t) (iThrust_1 + thrust_offset);
				control->normalizedForces[2] = (uint16_t) (iThrust_2 + thrust_offset);
				control->normalizedForces[3] = (uint16_t) (iThrust_3 + thrust_offset);
			}
		}
	}
}

PARAM_GROUP_START(paramNN)
PARAM_ADD_CORE(PARAM_UINT16, thrust_offset, &thrust_offset)
PARAM_GROUP_STOP(paramNN)

LOG_GROUP_START(logNN)

LOG_ADD(LOG_FLOAT, set_x, &setpoint_array[0])
LOG_ADD(LOG_FLOAT, set_y, &setpoint_array[1])
LOG_ADD(LOG_FLOAT, set_z, &setpoint_array[2])

LOG_ADD(LOG_FLOAT, set_o_roll, &setpoint_array[3])
LOG_ADD(LOG_FLOAT, set_o_pitch, &setpoint_array[4])
LOG_ADD(LOG_FLOAT, set_o_yaw, &setpoint_array[5])

LOG_ADD(LOG_FLOAT, set_vx, &setpoint_array[6])
LOG_ADD(LOG_FLOAT, set_vy, &setpoint_array[7])
LOG_ADD(LOG_FLOAT, set_vz, &setpoint_array[8])

#ifdef TOF_ENABLE
	#ifdef ENABLE_4X4_CONTROLLER
		LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
		LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
		LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
		LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])
	#else
		LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
		LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
		LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
		LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])
		LOG_ADD(LOG_FLOAT, obs5, &obstacle_inputs[4])
		LOG_ADD(LOG_FLOAT, obs6, &obstacle_inputs[5])
		LOG_ADD(LOG_FLOAT, obs7, &obstacle_inputs[6])
		LOG_ADD(LOG_FLOAT, obs8, &obstacle_inputs[7])
	#endif
#endif

LOG_GROUP_STOP(logNN)