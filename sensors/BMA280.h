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

#ifndef BMA280_H_
#define BMA280_H_

/* Register Map BMA280
http://www.mouser.com/ds/2/783/BST-BMA280-DS000-11_published-786496.pdf
*/

#define BMA280_BGW_CHIPID      0x00
#define BMA280_ACCD_X_LSB      0x02
#define BMA280_ACCD_X_MSB      0x03
#define BMA280_ACCD_Y_LSB      0x04
#define BMA280_ACCD_Y_MSB      0x05
#define BMA280_ACCD_Z_LSB      0x06
#define BMA280_ACCD_Z_MSB      0x07
#define BMA280_ACCD_TEMP       0x08
#define BMA280_INT_STATUS_0    0x09
#define BMA280_INT_STATUS_1    0x0A
#define BMA280_INT_STATUS_2    0x0B
#define BMA280_INT_STATUS_3    0x0C
#define BMA280_FIFO_STATUS     0x0E
#define BMA280_PMU_RANGE       0x0F
#define BMA280_PMU_BW          0x10
#define BMA280_PMU_LPW         0x11
#define BMA280_PMU_LOW_NOISE   0x12
#define BMA280_ACCD_HBW        0x13
#define BMA280_BGW_SOFTRESET   0x14
#define BMA280_INT_EN_0        0x16
#define BMA280_INT_EN_1        0x17
#define BMA280_INT_EN_2        0x18
#define BMA280_INT_MAP_0       0x19
#define BMA280_INT_MAP_1       0x1A
#define BMA280_INT_MAP_2       0x1B
#define BMA280_INT_SRC         0x1E
#define BMA280_INT_OUT_CTRL    0x20
#define BMA280_INT_RST_LATCH   0x21
#define BMA280_INT_0           0x22
#define BMA280_INT_1           0x23
#define BMA280_INT_2           0x24
#define BMA280_INT_3           0x25
#define BMA280_INT_4           0x26
#define BMA280_INT_5           0x27
#define BMA280_INT_6           0x28
#define BMA280_INT_7           0x29
#define BMA280_INT_8           0x2A
#define BMA280_INT_9           0x2B
#define BMA280_INT_A           0x2C
#define BMA280_INT_B           0x2D
#define BMA280_INT_C           0x2E
#define BMA280_INT_D           0x2F
#define BMA280_FIFO_CONFIG_0   0x30
#define BMA280_PMU_SELF_TEST   0x32
#define BMA280_TRIM_NVM_CTRL   0x33
#define BMA280_BGW_SPI3_WDT    0x34
#define BMA280_OFC_CTRL        0x36
#define BMA280_OFC_SETTING     0x37
#define BMA280_OFC_OFFSET_X    0x38
#define BMA280_OFC_OFFSET_Y    0x39
#define BMA280_OFC_OFFSET_Z    0x3A
#define BMA280_TRIM_GP0        0x3B
#define BMA280_TRIM_GP1        0x3C
#define BMA280_FIFO_CONFIG_1   0x3E
#define BMA280_FIFO_DATA       0x3F

#define AFS_2G  0x02
#define AFS_4G  0x05
#define AFS_8G  0x08
#define AFS_16G 0x0C

#define BW_7_81Hz  0x08  // 15.62 Hz sample rate, etc
#define BW_15_63Hz 0x09
#define BW_31_25Hz 0x0A
#define BW_62_5Hz  0x0B
#define BW_125Hz   0x0C  // 250 Hz sample rate
#define BW_250Hz   0x0D
#define BW_500Hz   0x0E
#define BW_1000Hz  0x0F  // 2 kHz sample rate == unfiltered data

#define normal_Mode      0x00  //define power modes
#define deepSuspend_Mode 0x01
#define lowPower_Mode    0x02
#define suspend_Mode     0x04

#define sleep_0_5ms   0x05  // define sleep duration in low power modes
#define sleep_1ms     0x06
#define sleep_2ms     0x07
#define sleep_4ms     0x08
#define sleep_6ms     0x09
#define sleep_10ms    0x0A
#define sleep_25ms    0x0B
#define sleep_50ms    0x0C
#define sleep_100ms   0x0D
#define sleep_500ms   0x0E
#define sleep_1000ms  0x0F

#ifndef ASSERT(expr)
#define ASSERT(expr) 
#endif

#ifdef __cplusplus
extern "C" {
#endif

    #include "../drivers/i2c.h"

    /**@brief BMA280 accelerometer's values structure.
     *        Contain values for each axis (x,y,z).
     */
    typedef struct BMA280_accel_values_s {
        uint16_t x;
        uint16_t y;
        uint16_t z;
    } BMA280_accel_values_t;


    /**@brief BMA280 base configuration structure.
     */
    typedef struct BMA280_config_s {
        uint8_t ascale;
        uint8_t BW;
        uint8_t power_mode;
        uint8_t sleep_dur;
    } BMA280_config_t;
    

    /**@brief Function for BMA280 sensor initialization.
     *
     * @param[in] config      Base sensor configuration.
     * @param[in] i2c         Instance of I2C driver (platform depended).
     * @param[in] address     Sensor's I2C address.
     */
    i2c_error_t BMA280_init(const BMA280_config_t * config, const i2c_instance_t * i2c, const uint8_t address);


    /**@brief Function for setting sensor's interrupt to detect any motion.
     *
     * @param[in] i2c         Instance of I2C driver (platform depended).
     * @param[in] address     Sensor's I2C address.
     */
    void BMA280_set_motion_detect_mode(const i2c_instance_t * i2c, const uint8_t address);


    /**@brief Function for reading accelerometer's values.
     *
     * @param[in] i2c         Instance of I2C driver (platform depended).
     * @param[in] address     Sensor's I2C address.
     * @param[out] dest       Pointer to buffer (sizeof BMA280_accel_values_t).
     */
    i2c_error_t BMA280_get_data(const i2c_instance_t * i2c, const uint8_t address, BMA280_accel_values_t * dest);


    /**@brief Function for BMA280 calibration.
     *
     * @param[in] i2c         Instance of I2C driver (platform depended).
     * @param[in] address     Sensor's I2C address.
     */
    void BMA280_calibrate(const i2c_instance_t * i2c, const uint8_t address);


#ifdef __cplusplus
}
#endif

#endif /* BMA280_H_ */
