/* Register Map BMA280
http://www.mouser.com/ds/2/783/BST-BMA280-DS000-11_published-786496.pdf
*/

#ifndef BMA280_H
#define BMA280_H

#include "../drivers/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AFS_2G           0x02
#define AFS_4G           0x05
#define AFS_8G           0x08
#define AFS_16G          0x0C

#define BW_7_81Hz        0x08  // 15.62 Hz sample rate, etc
#define BW_15_63Hz       0x09
#define BW_31_25Hz       0x0A
#define BW_62_5Hz        0x0B
#define BW_125Hz         0x0C  // 250 Hz sample rate
#define BW_250Hz         0x0D
#define BW_500Hz         0x0E
#define BW_1000Hz        0x0F  // 2 kHz sample rate == unfiltered data

#define normal_Mode      0x00  //define power modes
#define deepSuspend_Mode 0x01
#define lowPower_Mode    0x02
#define suspend_Mode     0x04

#define sleep_0_5ms      0x05  // define sleep duration in low power modes
#define sleep_1ms        0x06
#define sleep_2ms        0x07
#define sleep_4ms        0x08
#define sleep_6ms        0x09
#define sleep_10ms       0x0A
#define sleep_25ms       0x0B
#define sleep_50ms       0x0C
#define sleep_100ms      0x0D
#define sleep_500ms      0x0E
#define sleep_1000ms     0x0F    

#ifndef ASSERT
#define ASSERT(expr) 
#endif

/**@brief bma280 accelerometer's values structure.
 *        Contain values for each axis (x,y,z).
 */
typedef struct bma280_accel_values_s {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} bma280_accel_values_t;

/**@brief bma280 base configuration structure.
 */
typedef struct bma280_config_s {
    uint8_t ascale;
    uint8_t BW;
    uint8_t power_mode;
    uint8_t sleep_dur;
} bma280_config_t;

/**@brief Function for bma280 sensor initialization.
 *
 * @param[in] config      Base sensor configuration.
 * @param[in] i2c         Instance of I2C driver (platform depended).
 * @param[in] address     Sensor's I2C address.
 */
i2c_error_t bma280_init(const bma280_config_t * config, const i2c_instance_t * i2c, const uint8_t address);

/**@brief Function for setting sensor's interrupt to detect any motion in slow mode.
 *
 * @param[in] i2c         Instance of I2C driver (platform depended).
 * @param[in] address     Sensor's I2C address.
 */
void bma280_set_interrupt_detect_mode_slow(const i2c_instance_t * i2c, const uint8_t address);

/**@brief Function for reading accelerometer's values.
 *
 * @param[in] i2c         Instance of I2C driver (platform depended).
 * @param[in] address     Sensor's I2C address.
 * @param[out] dest       Pointer to buffer (sizeof bma280_accel_values_t).
 */
i2c_error_t bma280_get_data(const i2c_instance_t * i2c, const uint8_t address, bma280_accel_values_t * dest);

/**@brief Function for bma280 calibration.
 *
 * @param[in] i2c         Instance of I2C driver (platform depended).
 * @param[in] address     Sensor's I2C address.
 */
void bma280_calibrate(const i2c_instance_t * i2c, const uint8_t address);

/**
 * @brief Handler for bma280 interrupt.
 */
void bma280_interrupt_handler();

/**
 * @brief Check if bma280 was interrupt triggered.
 */
bool bma280_is_interrupt_detected(void);

#ifdef __cplusplus
}
#endif

#endif /* BMA280_H */
