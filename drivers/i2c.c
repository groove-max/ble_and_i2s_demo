#include "i2c.h"

// Indicates if operation on TWI has ended.
static volatile bool m_xfer_done = false;


void i2c_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:            
            m_xfer_done = true;
            break;

        default:
            break;
    }
}


ret_code_t i2c_write_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, const uint8_t data)
{
    ASSERT(i2c);

    ret_code_t err_code;

    uint8_t packet[2];
    packet[0] = sub_address;
    packet[1] = data;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, &packet[0], 2, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();

    return NRF_SUCCESS;
}


ret_code_t i2c_write_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t * data, const uint8_t n_bytes)
{
    ASSERT(i2c);
    ASSERT(data);
    
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, data, n_bytes, false);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();

    return NRF_SUCCESS;
}


ret_code_t i2c_read_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * data)
{
    ASSERT(i2c);
    ASSERT(data);

    ret_code_t err_code;
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(i2c, address, data, 1);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();

    return NRF_SUCCESS;
}


ret_code_t i2c_read_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * dest, const uint8_t n_bytes)
{
    ASSERT(i2c);
    ASSERT(dest);

    ret_code_t err_code;
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();
        
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(i2c, address, dest, n_bytes);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_TRANSFER_DONE();

    return NRF_SUCCESS;
}
