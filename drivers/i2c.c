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
    if (i2c == NULL)
    {
      return NRF_ERROR_NULL;
    }

    uint8_t packet[2];
    
    packet[0] = sub_address;
    packet[1] = data;
    
    ret_code_t err_code = 0;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, &packet[0], 2, true);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }

    while (m_xfer_done == false); //wait until end of transfer

    return NRF_SUCCESS;
}


ret_code_t i2c_write_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t * data, const uint8_t n_bytes)
{
    if (i2c == NULL || data == NULL)
    {
      return NRF_ERROR_NULL;
    }

    ret_code_t err_code = 0;
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, data, n_bytes, false);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }

    while (m_xfer_done == false); //wait until end of transfer

    return NRF_SUCCESS;
}


ret_code_t i2c_read_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * data)
{
    if (i2c == NULL || data == NULL)
    {
      return NRF_ERROR_NULL;
    }

    ret_code_t err_code = 0;
    
    uint8_t value;
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }
    
    while (m_xfer_done == false); //wait until end of transfer
    
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(i2c, address, &value, 1);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }
    
    while (m_xfer_done == false);

    *data = value;

    return NRF_SUCCESS;
}


ret_code_t i2c_read_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * dest, const uint8_t n_bytes)
{
    if (i2c == NULL || dest == NULL)
    {
      return NRF_ERROR_NULL;
    }

    ret_code_t err_code = 0;
    
    m_xfer_done = false;
    
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }
    
    while (m_xfer_done == false);
        
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(i2c, address, dest, n_bytes);
    if (err_code != NRF_SUCCESS) 
    {
        return err_code;
    }

    while (m_xfer_done == false);

    return NRF_SUCCESS;
}
