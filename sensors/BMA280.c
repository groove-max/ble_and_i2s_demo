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

#define BMA280_ADDRESS  0x18  // if ADO is 0 (default)

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


void BMA280_Init_Interrupts( void )
{
     writeByte(BMA280_ADDRESS, BMA280_INT_EN_1,  0x10);               // set data ready interrupt (bit 4) 
     writeByte(BMA280_ADDRESS, BMA280_INT_MAP_1, 0x01);               // map data ready interrupt to INT1 (bit 0) 
     writeByte(BMA280_ADDRESS, BMA280_INT_EN_0,  0x01 | 0x02 | 0x04); // set slope_en_x, slope_en_y, slope_en_z
     writeByte(BMA280_ADDRESS, BMA280_INT_RST_LATCH,  0x02 | 0x80);   // set temporary, 500 ms and reset_int to the interrupt mode
     writeByte(BMA280_ADDRESS, BMA280_INT_6,  0x08);                  // set the threshold definition for the any-motion interrupt
     writeByte(BMA280_ADDRESS, BMA280_INT_MAP_2, 0x04);               // set int2_slope
     writeByte(BMA280_ADDRESS, BMA280_INT_OUT_CTRL, 0x04 | 0x01);     // interrupts push-pull, active HIGH (bits 0:3) 
}


void BMA280_Turn_On( void )
{
    uint8_t Ascale      = AFS_2G;
    uint8_t BW          = BW_7_81Hz;
    uint8_t power_Mode  = lowPower_Mode;
    uint8_t sleep_dur   = sleep_500ms;

    // set full-scale range
    writeByte(BMA280_ADDRESS, BMA280_PMU_RANGE, Ascale);    
    // set bandwidth (and thereby sample rate)
    writeByte(BMA280_ADDRESS, BMA280_PMU_BW, BW);         
    // set power mode and sleep duration
    writeByte(BMA280_ADDRESS, BMA280_PMU_LPW, power_Mode << 5 | sleep_dur << 1); 

}   

void BMA280_Get_Data(BMA280_Accel_Values * dest)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(BMA280_ADDRESS, BMA280_ACCD_X_LSB, rawData, 6);  // Read the 6 raw data registers into data array
    dest->x = ((int16_t)rawData[1] << 8) | rawData[0];         // Turn the MSB and LSB into a signed 14-bit value
    dest->y = ((int16_t)rawData[3] << 8) | rawData[2];
    dest->z = ((int16_t)rawData[5] << 8) | rawData[4];
}

void BMA280_Calibrate( void )
{
    //must be in normal power mode, and set to +/- 2g
    SEGGER_RTT_printf(0, "Hold flat and motionless for bias calibration\n");
    
    nrf_delay_ms(3000);
    
    uint8_t rawData[2];    // x/y/z accel register data stored here
    float FCres = 7.8125f; // fast compensation offset mg/LSB
     
    writeByte(BMA280_ADDRESS, BMA280_OFC_SETTING, 0x20 | 0x01); // set target data to 0g, 0g, and +1g, cutoff at 1% of bandwidth
    
    writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x20); // x-axis calibration
    while(!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL))) { }; // wait for calibration completion
    
    writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x40); // y-axis calibration
    while(!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL))) { }; // wait for calibration completion
    
    writeByte(BMA280_ADDRESS, BMA280_OFC_CTRL, 0x60); // z-axis calibration
    while(!(0x10 & readByte(BMA280_ADDRESS, BMA280_OFC_CTRL))) { }; // wait for calibration completion

    readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_X, &rawData[0], 2);
    int16_t offsetX = ((int16_t)rawData[1] << 8) | 0x00;
    
    readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_Y, &rawData[0], 2);
    int16_t offsetY = ((int16_t)rawData[1] << 8) | 0x00;
    
    readBytes(BMA280_ADDRESS, BMA280_OFC_OFFSET_Z, &rawData[0], 2);
    int16_t offsetZ = ((int16_t)rawData[1] << 8) | 0x00;
    
    SEGGER_RTT_printf(0, "Bias calibration completed\n");
    SEGGER_RTT_printf(0, "x-axis offset = %d mg\n", (int16_t)(100.0f*(float)offsetX*FCres/256.0f));
    SEGGER_RTT_printf(0, "y-axis offset = %d mg\n", (int16_t)(100.0f*(float)offsetY*FCres/256.0f));
    SEGGER_RTT_printf(0, "z-axis offset = %d mg\n", (int16_t)(100.0f*(float)offsetZ*FCres/256.0f));

}

