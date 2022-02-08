
/**< Define BMA280 INT2 pin. */
#define BMA280_INT2_PIN  NRF_GPIO_PIN_MAP(1, 13)

/**< Define BMA280 TWI SCL pin. */
#define BMA280_SCL_PIN  NRF_GPIO_PIN_MAP(0, 27)

/**< Define BMA280 I2C SDA pin. */
#define BMA280_SDA_PIN  NRF_GPIO_PIN_MAP(0, 26)

#define BLE_MAXIMUM_CHARACTERISTICS_IN_SERVICE 2

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define I2C_BMA280_ADDRESS              0x18                                    /**< Define bma280 I2C address. */

#define DEVICE_BLE_NAME                 "BLE_and_I2S_demo"                      /**< Name of device. Will be included in the advertising data. */

#define TWI_INSTANCE_ID 0

#define TWI_IRQ_PRIORITY_LOW 3