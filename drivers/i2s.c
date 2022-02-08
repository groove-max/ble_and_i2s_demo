#include "i2s.h"

static struct i2s_instance_s {
    uint32_t buffer_tx[2][I2S_DATA_BLOCK_WORDS];                          /**< I2S TX buffer. */
    uint32_t buffer_rx[2][I2S_DATA_BLOCK_WORDS];                          /**< I2S RX buffer. */
    uint32_t offset;                                                      /**< Melody's array offset when writing to buffer. */
    uint32_t * volatile block_to_fill;                                    /**< Pointer to buffer, which will be written. */
    const uint8_t * active_melody;                                        /**< Pointer for active melody's const array. */
    uint32_t melody_array_size;                                           /**< Will written size of melody's array. */
} m_i2s_instance;

static void i2s_prepare_tx_data(uint32_t*);
__STATIC_INLINE void i2s_data_handler(nrf_drv_i2s_buffers_t const*, uint32_t);

ret_code_t i2s_init(nrf_drv_i2s_config_t const * config)
{
    ret_code_t err_code;

    memset(&m_i2s_instance, 0, sizeof(m_i2s_instance));

    err_code = nrf_drv_i2s_init(config, i2s_data_handler);
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}

void i2s_deinit(void)
{
    nrf_drv_i2s_uninit();
}

ret_code_t i2s_start_playing(const uint8_t * const p_active_melody, const uint32_t melody_arr_size)
{
    ASSERT(p_active_melody);
    ASSERT(melody_arr_size);

    // Check if offset is not empty. If not, then playback already is in progress.
    // Return no error.
    if(m_i2s_instance.offset > 0) 
    {
        return NRF_SUCCESS;
    }

    m_i2s_instance.active_melody     = p_active_melody;
    m_i2s_instance.melody_array_size = melody_arr_size;

    i2s_prepare_tx_data(m_i2s_instance.buffer_tx[0]);

    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = m_i2s_instance.buffer_tx[0],
        .p_rx_buffer = m_i2s_instance.buffer_rx[0],
    };

    ret_code_t err_code;
    err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}

void i2s_stop_playing(void)
{
    m_i2s_instance.block_to_fill = NULL;
    m_i2s_instance.offset        = 0;

    nrf_drv_i2s_stop();
}

bool i2s_was_playing_finished(void) 
{
    return m_i2s_instance.offset ? false : true;
}

void i2s_handle(void)
{
    if(m_i2s_instance.block_to_fill != NULL) 
    {
        if(m_i2s_instance.offset < m_i2s_instance.melody_array_size) 
        {
            i2s_prepare_tx_data(m_i2s_instance.block_to_fill);
            m_i2s_instance.block_to_fill = NULL;
        }
        else 
        {
            i2s_stop_playing();
        }
    }
}

/**@brief Function for preparing buffer to send via i2s.
 *
 * @param[in] p_block  Pointer to container which will be send. Each data word contains two 16-bit samples.
 */
static void i2s_prepare_tx_data(uint32_t * p_block)
{
    ASSERT(p_block);

    uint16_t i = 0;
    bool is_end = false;

    /* Clear pcm buffer */
    memset(p_block, 0, I2S_DATA_BLOCK_WORDS);

    /* Fill pcm buffer with nessessary part of melody's array */
    while( i < I2S_DATA_BLOCK_WORDS && !is_end ) 
    {
        uint32_t * p_word = &p_block[i];
        uint8_t j;
        for(j = 0; j < sizeof(uint32_t)/sizeof(uint8_t); j++) 
        {
            if(m_i2s_instance.offset >= m_i2s_instance.melody_array_size) 
            {
                is_end = true;
                break;
            }
            else 
            {
                ((uint8_t *)p_word)[j] = m_i2s_instance.active_melody[m_i2s_instance.offset++];
            }
        }
        i++;
    }
}

/**
 * @brief I2S driver data handler.
 */
__STATIC_INLINE void i2s_data_handler(nrf_drv_i2s_buffers_t const* p_released, uint32_t status)
{
    ASSERT(p_released);

    ret_code_t err_code;

    if(!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    if (!p_released->p_rx_buffer)
    {
        nrf_drv_i2s_buffers_t const next_buffer = {
            .p_tx_buffer = m_i2s_instance.buffer_tx[1],
	    .p_rx_buffer = m_i2s_instance.buffer_rx[1]
        };
        err_code = nrf_drv_i2s_next_buffers_set(&next_buffer);
        APP_ERROR_CHECK(err_code);

        m_i2s_instance.block_to_fill = m_i2s_instance.buffer_tx[1];
    } else 
    {
        err_code = nrf_drv_i2s_next_buffers_set(p_released);
        APP_ERROR_CHECK(err_code);

        m_i2s_instance.block_to_fill = (uint32_t *)p_released->p_tx_buffer;
    }

}
