#ifndef I2C_H__
#define I2C_H__

#define __LOG_PRINTF    NRF_LOG_INFO
#define I2C_NO_ERROR    NRF_SUCCESS

#define CHECK_ERROR_RETURN(error) if (error != NRF_SUCCESS) return error

#define I2C_WAIT_FOR_TRANSFER_DONE() while (m_xfer_done == false)

#ifdef __cplusplus
extern "C" {
#endif

    #include "nrf_assert.h"
    #include "nrf_drv_twi.h"
    #include "app_error.h"
    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
    #include "nrf_log_default_backends.h"

    typedef nrf_drv_twi_t   i2c_instance_t;
    typedef ret_code_t      i2c_error_t;


    /**@brief I2C Handler.
     */
    void i2c_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);


    /**@brief Function for writing one byte via I2C.
     *
     * @param[in] i2c         I2C instance.
     * @param[in] address     I2C destination address.
     * @param[in] sub_address I2C register address.
     * @param[in] data        Byte which will be send.
     */
    ret_code_t i2c_write_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, const uint8_t data);


    /**@brief Function for writing array of bytes via I2C.
     *
     * @param[in] i2c         I2C instance.
     * @param[in] address     I2C destination address.
     * @param[in] data        Pointer to array of bytes which will be send.
     * @param[in] n_bytes     Count of bytes.
     */
    ret_code_t i2c_write_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t * data, const uint8_t n_bytes);


    /**@brief Function for reading array of bytes via I2C.
     *
     * @param[in] i2c         I2C instance.
     * @param[in] address     I2C destination address.
     * @param[in] sub_address I2C register address.
     * @param[out] dest       Pointer to buffer of received data.
     * @param[in] n_bytes     Max buffer size.
     */
    ret_code_t i2c_read_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * dest, const uint8_t n_bytes);


    /**@brief Function for reading one byte via I2C.
     *
     * @param[in] i2c         I2C instance.
     * @param[in] address     I2C destination address.
     * @param[in] sub_address I2C register address.
     * @param[out] data       Pointer to one-byte buffer.
     */
    ret_code_t i2c_read_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * data);
    

#ifdef __cplusplus
}
#endif

#endif /* I2C_H__ */
