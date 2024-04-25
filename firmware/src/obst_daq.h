#ifndef OBST_DAQ_H
#define OBST_DAQ_H

// Custom Includes
#include "vl53l5cx_api.h"
#include "network_evaluate_tof.h"
#include "stabilizer_types.h"

#define EXPANDER_ADDR 0x20

/**
 * @brief Initializes all the tof sensors.
 * 
 * @return uint8_t 
 */
uint8_t tof_init(uint8_t* sensor_status, VL53L5CX_Configuration *tof_array, uint16_t *tof_addresses);
/**
 * @brief 
 * 
 * @return bool: Indicates that the measurement is now stale. 
 */

bool process_obst(const state_t *state, float *obstacle_inputs, uint16_t *tof_input, uint8_t *tof_status);
/**
 * @brief Collects ToF matrix data. 
 * 
 * @return bool: update on measurement to track stale data.
 */
bool tof_task(VL53L5CX_Configuration *tof_array, VL53L5CX_ResultsData* ranging_data, uint8_t* sensor_status, uint16_t *tof_input, uint8_t *tof_status);
bool I2C_expander_set_pin(uint8_t pin_number, uint8_t pin_value);
#endif