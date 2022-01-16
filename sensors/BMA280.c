/* Copyright (c) 2017, Stanford University
 * All rights reserved.
 * 
 * The point of contact for the MENTAID wearables dev team is 
 * Jan Liphardt (jan.liphardt@stanford.edu)
 * 
 * The code is modified from a reference implementation by Kris Winer
 * (tleracorp@gmail.com)
 * Copyright (c) 2017, Tlera Corporation
 * The license terms of the TLERACORP material are: 
 * "Library may be used freely and without limit with attribution."
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of STANFORD UNIVERSITY nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY STANFORD UNIVERSITY "AS IS" AND ANY EXPRESS 
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL STANFORD UNIVERSITY OR ITS CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "BMA280.h"


void BMA280_set_motion_detect_mode(const i2c_instance_t * i2c, const uint8_t address)
{
    i2c_write_byte(i2c, address, BMA280_INT_EN_1,  0x10);               // set data ready interrupt (bit 4) 
    i2c_write_byte(i2c, address, BMA280_INT_MAP_1, 0x01);               // map data ready interrupt to INT1 (bit 0) 
    i2c_write_byte(i2c, address, BMA280_INT_EN_0,  0x01 | 0x02 | 0x04); // set slope_en_x, slope_en_y, slope_en_z
    i2c_write_byte(i2c, address, BMA280_INT_RST_LATCH,  0x02 | 0x80);   // set temporary, 500 ms and reset_int to the interrupt mode
    i2c_write_byte(i2c, address, BMA280_INT_6,  0x08);                  // set the threshold definition for the any-motion interrupt
    i2c_write_byte(i2c, address, BMA280_INT_MAP_2, 0x04);               // set int2_slope
    i2c_write_byte(i2c, address, BMA280_INT_OUT_CTRL, 0x04 | 0x01);     // interrupts push-pull, active HIGH (bits 0:3) 
}


i2c_error_t BMA280_init(const BMA280_config_t * config, const i2c_instance_t * i2c, const uint8_t address)
{
    ASSERT(config);

    i2c_error_t err_code;

    // set full-scale range
    err_code = i2c_write_byte(i2c, address, BMA280_PMU_RANGE, config->ascale);
    CHECK_ERROR_RETURN(err_code);
    // set bandwidth (and thereby sample rate)
    i2c_write_byte(i2c, address, BMA280_PMU_BW, config->BW);
    CHECK_ERROR_RETURN(err_code);
    // set power mode and sleep duration
    i2c_write_byte(i2c, address, BMA280_PMU_LPW, config->power_mode << 5 | config->sleep_dur << 1);
    CHECK_ERROR_RETURN(err_code);

    return I2C_NO_ERROR;
}   

i2c_error_t BMA280_get_data(const i2c_instance_t * i2c, const uint8_t address, BMA280_accel_values_t * dest)
{
    ASSERT(dest);

    i2c_error_t err_code = I2C_NO_ERROR;

    uint8_t rawData[6];  // x/y/z accel register data stored here
    err_code = i2c_read_bytes(i2c, address, BMA280_ACCD_X_LSB, rawData, 6);  // Read the 6 raw data registers into data array
    if(err_code == I2C_NO_ERROR)
    {
        dest->x = ((int16_t)rawData[1] << 8) | rawData[0];         // Turn the MSB and LSB into a signed 14-bit value
        dest->y = ((int16_t)rawData[3] << 8) | rawData[2];
        dest->z = ((int16_t)rawData[5] << 8) | rawData[4];
    }

    return err_code;
}

void BMA280_calibrate(const i2c_instance_t * i2c, const uint8_t address)
{
    //must be in normal power mode, and set to +/- 2g
    #ifdef __LOG_PRINTF
        __LOG_PRINTF("Hold flat and motionless for bias calibration");
    #endif

    uint8_t rd_byte;    
    uint8_t rawData[2];    // x/y/z accel register data stored here
    float FCres = 7.8125f; // fast compensation offset mg/LSB
    

    i2c_write_byte(i2c, address, BMA280_OFC_SETTING, 0x20 | 0x01); // set target data to 0g, 0g, and +1g, cutoff at 1% of bandwidth
    
    i2c_write_byte(i2c, address, BMA280_OFC_CTRL, 0x20); // x-axis calibration

    do {
        i2c_read_byte(i2c, address, BMA280_OFC_CTRL, &rd_byte);
    } while(!(0x10 & rd_byte)); // wait for calibration completion
    
    i2c_write_byte(i2c, address, BMA280_OFC_CTRL, 0x40); // y-axis calibration
    do {
        i2c_read_byte(i2c, address, BMA280_OFC_CTRL, &rd_byte);
    } while(!(0x10 & rd_byte)); // wait for calibration completion
    
    i2c_write_byte(i2c, address, BMA280_OFC_CTRL, 0x60); // z-axis calibration
    do {
        i2c_read_byte(i2c, address, BMA280_OFC_CTRL, &rd_byte);
    } while(!(0x10 & rd_byte)); // wait for calibration completion

    i2c_read_bytes(i2c, address, BMA280_OFC_OFFSET_X, &rawData[0], 2);
    int16_t offsetX = ((int16_t)rawData[1] << 8) | 0x00;
    
    i2c_read_bytes(i2c, address, BMA280_OFC_OFFSET_Y, &rawData[0], 2);
    int16_t offsetY = ((int16_t)rawData[1] << 8) | 0x00;
    
    i2c_read_bytes(i2c, address, BMA280_OFC_OFFSET_Z, &rawData[0], 2);
    int16_t offsetZ = ((int16_t)rawData[1] << 8) | 0x00;
    
    #ifdef __LOG_PRINTF
        __LOG_PRINTF("Bias calibration completed");
        __LOG_PRINTF("x-axis offset = %d mg", (int16_t)(100.0f*(float)offsetX*FCres/256.0f));
        __LOG_PRINTF("y-axis offset = %d mg", (int16_t)(100.0f*(float)offsetY*FCres/256.0f));
        __LOG_PRINTF("z-axis offset = %d mg", (int16_t)(100.0f*(float)offsetZ*FCres/256.0f));
    #endif
}

