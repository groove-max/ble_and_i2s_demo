#ifndef I2S_H
#define I2S_H

#include <string.h>

#include "nrf_drv_i2s.h"
#include "app_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_DATA_BLOCK_WORDS 1024

/**@brief Function for i2s initialization.
 *
 * @param[in] config                Config init structure.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE The driver was already initialized.
 * @retval NRFX_ERROR_INVALID_PARAM The requested combination of configuration
 *                                  options is not allowed by the I2S peripheral.
 */
ret_code_t i2s_init(nrf_drv_i2s_config_t const * config);

/**@brief Function for i2s deinitialization.
 */
void i2s_deinit(void);

/**@brief Function for i2s start playing.
 *
 * @param[in] p_active_melody  Pointer to melody WAV array.
 * @param[in] melody_arr_size  Size of melody WAV array in bytes.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_INVALID_STATE 
 * @retval NRFX_ERROR_INVALID_ADDR  
*/
ret_code_t i2s_start_playing(const uint8_t * const p_active_melody, const uint32_t melody_arr_size);

/**@brief Function for i2s stop playing.
*/
void i2s_stop_playing(void);

/**@brief Function for checking if i2s playing was already finished.
 *
 * @retval true             If already finished.
 * @retval false            If still playing.
*/
bool i2s_was_playing_finished(void);

/**@brief Function for handling I2S. Need to be placed into loop, if you want to play.
 */
void i2s_handle(void);


#ifdef __cplusplus
}
#endif

#endif /* I2S_H */
