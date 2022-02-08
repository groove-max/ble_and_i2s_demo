#include "bma280.h"

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

void bma280_set_interrupt_detect_mode_slow(const i2c_instance_t * i2c, const uint8_t address)
{
    ASSERT(i2c);
    ASSERT(address);

    i2c_write_byte(i2c, address, BMA280_INT_EN_1,  0x10);               // set data ready interrupt (bit 4) 
    i2c_write_byte(i2c, address, BMA280_INT_MAP_1, 0x01);               // map data ready interrupt to INT1 (bit 0) 
    i2c_write_byte(i2c, address, BMA280_INT_EN_0,  0x01 | 0x02 | 0x04); // set slope_en_x, slope_en_y, slope_en_z
    i2c_write_byte(i2c, address, BMA280_INT_RST_LATCH,  0x02 | 0x80);   // set temporary, 500 ms and reset_int to the interrupt mode
    i2c_write_byte(i2c, address, BMA280_INT_6,  0x08);                  // set the threshold definition for the any-motion interrupt
    i2c_write_byte(i2c, address, BMA280_INT_MAP_2, 0x04);               // set int2_slope
    i2c_write_byte(i2c, address, BMA280_INT_OUT_CTRL, 0x04 | 0x01);     // interrupts push-pull, active HIGH (bits 0:3) 
}

i2c_error_t bma280_init(const bma280_init_t * config, const i2c_instance_t * i2c, const uint8_t address)
{
    ASSERT(config);
    ASSERT(i2c);
    ASSERT(address);

    i2c_error_t err_code;

    // set full-scale range
    err_code = i2c_write_byte(i2c, address, BMA280_PMU_RANGE, config->ascale);
    CHECK_ERROR_RETURN(err_code);

    // set bandwidth (and thereby sample rate)
    err_code = i2c_write_byte(i2c, address, BMA280_PMU_BW, config->BW);
    CHECK_ERROR_RETURN(err_code);

    // set power mode and sleep duration
    err_code = i2c_write_byte(i2c, address, BMA280_PMU_LPW, config->power_mode << 5 | config->sleep_dur << 1);
    CHECK_ERROR_RETURN(err_code);

    return I2C_NO_ERROR;
}   

i2c_error_t bma280_get_data(const i2c_instance_t * i2c, const uint8_t address, bma280_values_t * dest)
{
    ASSERT(i2c);
    ASSERT(address);
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

void bma280_calibrate(const i2c_instance_t * i2c, const uint8_t address)
{
    ASSERT(i2c);
    ASSERT(address);

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
