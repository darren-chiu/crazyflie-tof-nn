#include "obst_daq.h"
#include "i2cdev.h"


uint8_t tof_init(uint8_t* sensor_status, VL53L5CX_Configuration *tof_array, uint16_t *tof_addresses) {
    bool I2C_expander_status;

    // Set all the pins to be outputs. Setting all pins to 0, makes them all outputs.
    I2C_expander_status = i2cdevWriteByte(I2C1_DEV, EXPANDER_ADDR, 0x03, 0x00);
    if (I2C_expander_status == true) {
        DEBUG_PRINT("I2C Output Configuration: Pass"); 
    }
    // Disable all the sensors by setting the output of all pins to be 0.
    I2C_expander_status = i2cdevWriteByte(I2C1_DEV, EXPANDER_ADDR, 0x01, 0x00);
    if (I2C_expander_status == true) {
        DEBUG_PRINT("Disabled all ToF sensors."); 
    }

    for (int i =0; i < NUM_SENSORS; i++) {
        // Enable the i'th ToF sensor.
        I2C_expander_set_pin(i, 1);
        
        *sensor_status = vl53l5cx_init(&tof_array[i]);
        vTaskDelay(M2T(100));

        tof_array[i].platform = VL53L5CX_DEFAULT_I2C_ADDRESS;


        *sensor_status = vl53l5cx_set_i2c_address(&tof_array[i], tof_addresses[i]);
        DEBUG_PRINT("Set VL53L5CX [%i] Address\n", i);
        // The following depends on which configuration is used.
        #ifdef ENABLE_4X4_CONTROLLER
            // Sets the sensor to be 4x4.
            *sensor_status = vl53l5cx_set_resolution(&tof_array[i], VL53L5CX_RESOLUTION_4X4);
            if (*sensor_status == 0) {
                DEBUG_PRINT("VL53L5CX [%i] Initialize: Pass\n", i); 
            }
            *sensor_status = vl53l5cx_set_ranging_frequency_hz(&tof_array[i], 30);

            if (*sensor_status == 0) {
                DEBUG_PRINT("VL53L5CX [%i] Frequency Config: Pass\n", i); 
            } 
        #else
            // Sets the sensor to be 8x8.
            *sensor_status = vl53l5cx_set_resolution(&f_dev, VL53L5CX_RESOLUTION_8X8);
            if (*sensor_status == 0) {
                DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
            }
            *sensor_status = vl53l5cx_set_ranging_frequency_hz(&f_dev, 15);

            if (*sensor_status == 0) {
                DEBUG_PRINT("VL53L5CX Frequency Config: Pass\n"); 
            } 
        #endif

        if (sensor_status == 0) {
            DEBUG_PRINT("VL53L5CX [%i] Initialize: Pass\n", i); 
        }
        //Below function should be the last called in the init. 
        *sensor_status = vl53l5cx_start_ranging(&tof_array[i]);
        if (*sensor_status == 0) {
            DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
        } 
    }

    return *sensor_status;
}

bool process_obst(const state_t *state, float *obstacle_inputs, uint16_t *tof_input, uint8_t *tof_status) {
   	/**
   	 * NOTE: Use only the values of a specific column
     * The ToF lens flips the image plane vertically and horizontally
   	 */
    int network_index = 0;
    #ifdef ENABLE_4X4_CONTROLLER
		// ToF Measurements are noisy on takeoff.
		if (state->position.z > SAFE_HEIGHT) {
			int row_index = 4; //This denotes which row to start from. See note above.
            for (int j=0;j<NUM_SENSORS;j++) {
			    for (int i=0;i<4;i++) {				
                    int curr_index = row_index + i;
                    // Check if the pixels are valid 
                    if ((tof_status[j + curr_index] == 9) || (tof_status[j + curr_index] == 5)) {
                        float obst_cap;

                        obst_cap = tof_input[j + curr_index] * 1.0f;
                        obst_cap = obst_cap / 1000.0f;

                        if ((obst_cap > OBST_MAX)) {
                            obst_cap = OBST_MAX;
                        }
                        obstacle_inputs[network_index] = obst_cap;
                        // obstacle_inputs[i] = OBST_MAX; //Ablate inputs to NN
                    } else {
                        // DEBUG_PRINT("Invalid Reading!");
                        obstacle_inputs[network_index] = OBST_MAX;
                    }
                    network_index++;
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

    return true;
}

bool tof_task(VL53L5CX_Configuration *tof_array, VL53L5CX_ResultsData* ranging_data, uint8_t* sensor_status, 
				uint16_t *tof_input, uint8_t *tof_status) {
    for (int i = 0; i < NUM_SENSORS; i++) {
	    vl53l5cx_check_data_ready(&tof_array[i], sensor_status);

        if (*sensor_status){
            vl53l5cx_get_ranging_data(&tof_array[i], ranging_data);
            memcpy(&tof_input[i*OBST_DIM], (uint16_t *)(ranging_data[i].distance_mm[0]), OBST_DIM*2);
            // memcpy(tof_target, (uint8_t *)(&ranging_data->nb_target_detected[0]), OBST_DIM*OBST_DIM);
            memcpy(&tof_status[i*OBST_DIM], (uint8_t *)(ranging_data[i].target_status[0]), OBST_DIM);
        }
        
    }

    // Sets the "isStale" variable to false in order to indicate a fresh measurement is here. Yummy.
    return false;
}


/**
 * Function written by Vlad Niculescu
 * https://github.com/ETH-PBL/NanoSLAM/blob/main/stm32-app/src/tof-matrix/drivers/i2c_expander.c#L17
 */
bool I2C_expander_set_pin(uint8_t pin_number, uint8_t pin_value) {
    bool status = true;
    uint8_t reg_value;

    // Read current value of the output register
    status = status && i2cdevReadByte(I2C1_DEV, EXPANDER_ADDR,
                                      0x01, &reg_value);

    if (pin_value == 1)
        reg_value |= (1 << pin_number);
    else
        reg_value &= ~(1 << pin_number);

    // Update output register value
    status = status && i2cdevWriteByte(I2C1_DEV, EXPANDER_ADDR,
                                       0x01, reg_value);

    return status;
}