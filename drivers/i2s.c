#include "i2s.h"

static uint32_t ma_buffer_tx[2][I2S_DATA_BLOCK_WORDS];                          /**< I2S TX buffer. */
static uint32_t m_offset = 0;                                                   /**< Melody's array offset when writing to buffer. */
static uint32_t * volatile mp_block_to_fill = NULL;
static const uint8_t * mp_active_melody = NULL;                                 /**< Pointer for active melody's const array. */
static uint32_t ma_melody_array_size = 0;                                       /**< Will write size of melody's array. */


static void i2s_prepare_tx_data(uint32_t * p_block)
{
    ASSERT(p_block);

    // [each data word contains two 16-bit samples]
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
            if(m_offset >= ma_melody_array_size) 
            {
                is_end = true;
                break;
            }
            else 
            {
                ((uint8_t *)p_word)[j] = mp_active_melody[m_offset++];
            }
        }
        i++;
    }
}


static ret_code_t i2s_prepare_buffers_and_start_transfer(void)
{
    i2s_prepare_tx_data(ma_buffer_tx[0]);

    nrf_drv_i2s_buffers_t const initial_buffers = {
        .p_tx_buffer = ma_buffer_tx[0],
        .p_rx_buffer = NULL,
    };

    ret_code_t err_code;
    err_code = nrf_drv_i2s_start(&initial_buffers, I2S_DATA_BLOCK_WORDS, 0);
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}


ret_code_t i2s_start_playing(const uint8_t * const p_active_melody, const uint32_t melody_arr_size)
{
    ASSERT(p_active_melody);

    // Check if offset is not empty. If not, then playback already is in progress.
    // Return no error, because no need to call APP_ERROR_HANDLER.
    if(m_offset > 0) 
    {
        return NRF_SUCCESS;
    }

    mp_active_melody     = p_active_melody;
    ma_melody_array_size = melody_arr_size;

    ret_code_t err_code;
    err_code = i2s_prepare_buffers_and_start_transfer();
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}


void i2s_stop_playing(void)
{
    mp_block_to_fill = NULL;
    m_offset = 0;

    nrf_drv_i2s_stop();
}


void i2s_data_handler(nrf_drv_i2s_buffers_t const* p_released, uint32_t status)
{
    // 'nrf_drv_i2s_next_buffers_set' is called directly from the handler
    // each time next buffers are requested, so data corruption is not
    // expected.
    ASSERT(p_released);

    // When the handler is called after the transfer has been stopped
    // (no next buffers are needed, only the used buffers are to be
    // released), there is nothing to do.
    if(!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }

    // First call of this handler occurs right after the transfer is started.
    // No data has been transferred yet at this point, so there is nothing to
    // check. Only the buffers for the next part of the transfer should be
    // provided.
    if (!p_released->p_rx_buffer)
    {
        nrf_drv_i2s_buffers_t const next_buffer = {
            .p_tx_buffer = ma_buffer_tx[1],
	    .p_rx_buffer = NULL,
        };
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(&next_buffer));

        mp_block_to_fill = ma_buffer_tx[1];
    }
    else
    {
        // The driver has just finished accessing the buffers pointed by
        // 'p_released'. They can be used for the next part of the transfer
        // that will be scheduled now.
        APP_ERROR_CHECK(nrf_drv_i2s_next_buffers_set(p_released));

        // The pointer needs to be typecasted here, so that it is possible to
        // modify the content it is pointing to (it is marked in the structure
        // as pointing to constant data because the driver is not supposed to
        // modify the provided data).
        mp_block_to_fill = (uint32_t *)p_released->p_tx_buffer;
    }

}


void i2s_handle(void)
{
    if(mp_block_to_fill) 
    {
        if(m_offset < ma_melody_array_size) 
        {
            i2s_prepare_tx_data(mp_block_to_fill);
            mp_block_to_fill = NULL;
        }
        else 
        {
            i2s_stop_playing();
        }
    }
}
