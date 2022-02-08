#include "i2c.h"

// Indicates if operation on TWI has ended.
static volatile bool m_xfer_done = false;
static volatile uint32_t m_xfer_errs = NRF_SUCCESS;


static void i2c_handler(nrf_drv_twi_evt_t const*, void*);


ret_code_t i2c_init(const i2c_instance_t * i2c, const nrf_drv_twi_config_t const * i2c_config)
{
    ASSERT(i2c);
    ASSERT(i2c_config);

    ret_code_t err_code;
    err_code = nrf_drv_twi_init(i2c, i2c_config, i2c_handler, NULL);
    CHECK_ERROR_RETURN(err_code);

    nrf_drv_twi_enable(i2c);

    return NRF_SUCCESS;
    
}


ret_code_t i2c_write_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, const uint8_t data)
{
    ASSERT(i2c);

    ret_code_t err_code;

    uint8_t packet[2];
    packet[0] = sub_address;
    packet[1] = data;

    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_tx(i2c, address, &packet[0], 2, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);

    return NRF_SUCCESS;
}


ret_code_t i2c_write_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t * data, const uint8_t n_bytes)
{
    ASSERT(i2c);
    ASSERT(data);
    
    ret_code_t err_code;

    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_tx(i2c, address, data, n_bytes, false);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);

    return NRF_SUCCESS;
}


ret_code_t i2c_read_byte(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * data)
{
    ASSERT(i2c);
    ASSERT(data);

    ret_code_t err_code;
    
    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);
    
    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_rx(i2c, address, data, 1);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);

    return NRF_SUCCESS;
}


ret_code_t i2c_read_bytes(const i2c_instance_t * i2c, const uint8_t address, const uint8_t sub_address, uint8_t * dest, const uint8_t n_bytes)
{
    ASSERT(i2c);
    ASSERT(dest);

    ret_code_t err_code;
    
    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_tx(i2c, address, &sub_address, 1, true);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);
        
    m_xfer_done = false;
    m_xfer_errs = NRF_SUCCESS;
    err_code = nrf_drv_twi_rx(i2c, address, dest, n_bytes);
    CHECK_ERROR_RETURN(err_code);
    I2C_WAIT_FOR_XFER_DONE();
    CHECK_ERROR_RETURN(m_xfer_errs);

    return NRF_SUCCESS;
}


/**@brief I2C Handler.
 */

static void i2c_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    ASSERT(p_event);

    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            m_xfer_errs = NRF_SUCCESS;
            break;

        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            m_xfer_done = true;
            m_xfer_errs = NRF_ERROR_DRV_TWI_ERR_ANACK;
            break;

        case NRF_DRV_TWI_EVT_DATA_NACK:
            m_xfer_done = true;
            m_xfer_errs = NRF_ERROR_DRV_TWI_ERR_DNACK;
            break;

        default:
            break;
    }
}
