#ifndef I2S_H__
#define I2S_H__

#define I2S_DATA_BLOCK_WORDS 1024

#define CHECK_ERROR_RETURN(error) if (error != NRF_SUCCESS) return error

#ifdef __cplusplus
extern "C" {
#endif

    #include <string.h>
    
    #include "nrf_drv_i2s.h"
    #include "app_error.h"

    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
    #include "nrf_log_default_backends.h"

    
    /**@brief Function for preparing buffer to send via i2s.
     *
     * @param[in] p_block  Pointer to container which will be send.
     */
    static void i2s_prepare_tx_data(uint32_t * p_block);


    /**@brief Function for preparing i2s buffers and starting transfer.
     */
    static uint32_t i2s_prepare_buffers_and_start_transfer(void);


    /**@brief Function for i2s start playing.
     *
     * @param[in] p_active_melody  Pointer to melody WAV const array.
     * @param[in] melody_arr_size  Size of melody WAV const array.
    */
    ret_code_t i2s_start_playing(const uint8_t * const p_active_melody, const uint32_t melody_arr_size);


    /**@brief Function for i2s stop playing.
    */
    void i2s_stop_playing(void);


    /**
     * @brief I2S driver data handler.
     */
    void i2s_data_handler(nrf_drv_i2s_buffers_t const* p_released, uint32_t status);


    /**@brief Function for handling the I2S interrupt (main loop).
     */
    void i2s_handle(void);


#ifdef __cplusplus
}
#endif

#endif /* I2S_H__ */
