#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define DEVICE_BLE_NAME               "BLE_and_I2S_demo"                      /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL              MSEC_TO_UNITS(1000, UNIT_0_625_MS)      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define BLE_MAXIMUM_CHARS_IN_SERVICE  2                                       /**< Max count of characteristics in one service. */

#define BMA280_INT2_PIN               NRF_GPIO_PIN_MAP(1, 13)                 /**< Define BMA280 INT2 pin. */
#define BMA280_SCL_PIN                NRF_GPIO_PIN_MAP(0, 27)                 /**< Define BMA280 I2C SCL pin. */
#define BMA280_SDA_PIN                NRF_GPIO_PIN_MAP(0, 26)                 /**< Define BMA280 I2C SDA pin. */

#define BMA280_I2C_ADDRESS            0x18                                    /**< Define bma280 I2C address. */

#define DEAD_BEEF                     0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



/**@brief Macro for returning ERROR from function if supplied error code any other than NRF_SUCCESS.
 *
 * @param[in] ERR_CODE Error code supplied to return.
 */
#define CHECK_ERROR_RETURN(ERR_CODE)                        \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
            return LOCAL_ERR_CODE;                          \
        }                                                   \
    } while (0)

#endif