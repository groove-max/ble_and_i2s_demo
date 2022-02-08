#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_gpiote.h"
#include "app_timer.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "drivers/i2c.h"
#include "drivers/i2s.h"

#include "ble_manager/ble_manager.h"

#include "sensors/bma280.h"

#include "melodies/melody_1.h"
#include "melodies/melody_2.h"

/** @brief Available commands that can be received via i2s.
 */
typedef enum player_commands_e {
    I2S_PLAY_FIRST_MELODY  = 0x01,
    I2S_PLAY_SECOND_MELODY = 0x02,
    I2S_PLAY_STOP_PLAYING  = 0x03
} player_commands_t;

/** @brief Specify list of ble services we will use.
 * Last enum is count of services.
 */
enum main_ble_services_e {
    MAIN_BLE_CUSTOM_SERVICE,
    //
    MAIN_BLE_SERVICES_COUNT
};

/** @brief Specify list of ble characteristics we will create inside Custom Service.
 */
enum main_ble_custom_service_chars_e {
    MAIN_BLE_AUDIO_CHAR,
    MAIN_BLE_ACCEL_CHAR
};

/** @brief Specify UUIDs for services and characteristics.
 */

// The bytes are stored in little-endian format.
// 6ac131d4-6985-4b39-80e7-1740b2426d2d
static const ble_uuid128_t g_main_ble_services_base_uuids[] = {
    {0x2D, 0x6D, 0x42, 0xB2, 0x40, 0x17, 0xE7, 0x80, 0x39, 0x4B, 0x85, 0x69, 0xD4, 0x31, 0xC1, 0x6A}
};

static const uint16_t g_main_ble_services_uuids[] = {
    0x1400
};

static const uint8_t * g_main_ble_chars_names[] = {
    "Audio",
    "Accelerometer"
};

static const uint16_t g_main_ble_chars_uuids[] = {
    0x1401,
    0x1402
};

/** @brief Accelerometer Characteristic handles. */
static ble_gatts_char_handles_t gp_main_ble_accel_handles;

/** @brief Flag to detect bma280 interrupt. */
static volatile bool m_bma280_int_detected = true;

#define TWI_INSTANCE_ID       0
#define TWI_IRQ_PRIORITY_LOW  3

/** @brief Define our instances of BLE and i2c. */
BLE_MANAGER_DEF(m_ble_services, MAIN_BLE_SERVICES_COUNT);                       /**< BLE Manager instance. */
I2C_DEF(m_i2c_instance, TWI_INSTANCE_ID);                                       /**< I2C instance. */

/**@brief Forward declaration of some init functions. */
__STATIC_INLINE void log_init(void);
__STATIC_INLINE void timers_init(void);
__STATIC_INLINE void gpio_init(void);
__STATIC_INLINE void power_management_init(void);

/**@brief Function for handling write events to the Audio characteristic.
 *
 * @param[out] writable_char  Instance of BLE Characteristic.
 * @param[out] data           Buffer of received data.
 * @param[out] data_len       Size of data.
 */
__STATIC_INLINE void audio_write_handler(const ble_characteristic_t const * writable_char, const uint8_t * data, const uint16_t data_len)
{
    ASSERT(writable_char);
    ASSERT(data);
    ASSERT(data_len);

    ret_code_t        err_code;
    player_commands_t command = (player_commands_t) data[0];
     
    switch(command) 
    {
        case I2S_PLAY_STOP_PLAYING:
            NRF_LOG_DEBUG("Received i2s stop command: 0x%x", command);
            i2s_stop_playing();
            break;

        case I2S_PLAY_FIRST_MELODY:
            NRF_LOG_DEBUG("Received i2s play command: 0x%x", command);
            err_code = i2s_start_playing(melody_1, sizeof(melody_1));
            APP_ERROR_CHECK(err_code);
            break;

        case I2S_PLAY_SECOND_MELODY:
            NRF_LOG_DEBUG("Received i2s play command: 0x%x", command);
            err_code = i2s_start_playing(melody_2, sizeof(melody_2));
            APP_ERROR_CHECK(err_code);
            break;

        default:
            NRF_LOG_DEBUG("Command not recognized: 0x%x", command);
            break;
    }
}

/**@brief Function for initializing BLE Services.
 */
__STATIC_INLINE void ble_init(void)
{
    ret_code_t err_code;

    err_code = ble_manager_init(DEVICE_BLE_NAME); // Initialization
    APP_ERROR_CHECK(err_code);

    const ble_service_init_t ble_service = {
        .base_uuid     = g_main_ble_services_base_uuids[MAIN_BLE_CUSTOM_SERVICE],
        .uuid          = g_main_ble_services_uuids[MAIN_BLE_CUSTOM_SERVICE]
    };
    err_code = ble_manager_add_service(&m_ble_services[MAIN_BLE_CUSTOM_SERVICE], &ble_service); // Creating service
    APP_ERROR_CHECK(err_code);

    const ble_characteristic_init_t audio_char = {
        .char_name     = g_main_ble_chars_names[MAIN_BLE_AUDIO_CHAR],
        .char_uuid     = g_main_ble_chars_uuids[MAIN_BLE_AUDIO_CHAR],
        .is_readable   = false,
        .is_writeable  = true,
        .attr_len      = sizeof(player_commands_t),
        .write_handler = audio_write_handler
    };
    err_code = ble_manager_add_characteristic(&m_ble_services[MAIN_BLE_CUSTOM_SERVICE], &audio_char, NULL); // Added the first char
    APP_ERROR_CHECK(err_code);

    const ble_characteristic_init_t accel_char = {
        .char_name     = g_main_ble_chars_names[MAIN_BLE_ACCEL_CHAR],
        .char_uuid     = g_main_ble_chars_uuids[MAIN_BLE_ACCEL_CHAR],
        .is_readable   = true,
        .is_writeable  = false,
        .attr_len      = sizeof(bma280_values_t),
        .write_handler = NULL
    };
    err_code = ble_manager_add_characteristic(&m_ble_services[MAIN_BLE_CUSTOM_SERVICE], &accel_char, &gp_main_ble_accel_handles); // Added the second char
    APP_ERROR_CHECK(err_code);

    err_code = ble_manager_init_finalize(); // Finalize
    APP_ERROR_CHECK(err_code);
    err_code = ble_manager_advertising_start();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for sound(i2s) initialization.
 */
__STATIC_INLINE void snd_init(void)
{
    nrf_drv_i2s_config_t config = NRF_DRV_I2S_DEFAULT_CONFIG;

    config.sdin_pin     = NRFX_I2S_PIN_NOT_USED;
    config.mck_setup    = NRF_I2S_MCK_32MDIV31;
    config.ratio        = NRF_I2S_RATIO_64X;
    config.alignment    = NRF_I2S_ALIGN_LEFT;
    config.channels     = NRF_I2S_CHANNELS_LEFT;
    config.sample_width = NRF_I2S_SWIDTH_16BIT;

    ret_code_t err_code;
    err_code = i2s_init(&config);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the bma280 interrupt (main loop).
 */
__STATIC_INLINE void bma280_handle()
{
    if (m_bma280_int_detected == true) 
    {
        m_bma280_int_detected = false;

        ret_code_t      err_code;
        bma280_values_t result;

        err_code = bma280_get_data(&m_i2c_instance, BMA280_I2C_ADDRESS, &result);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("Cannot read values from bma280 sensor, error: %d", err_code);
            return;
        }

        NRF_LOG_DEBUG("X: %d, Y: %d, Z: %d", result.x, result.y, result.z);

        err_code = ble_manager_update_char_value(&m_ble_services[MAIN_BLE_CUSTOM_SERVICE], 
                                                 &gp_main_ble_accel_handles, 
                                                 &result, 
                                                 sizeof(result));
        if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE) 
        {
            NRF_LOG_DEBUG("Cannot update characteristic value, error: %d", err_code);
        }
    }
}



/**@brief Function for bma280 sensor initialization.
 */
__STATIC_INLINE void accelerometer_init(void)
{
    ret_code_t err_code;

    // Firstly need to init i2c instance.
    const nrf_drv_twi_config_t i2c_config = 
    {
       .scl                = BMA280_SCL_PIN,
       .sda                = BMA280_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = TWI_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = i2c_init(&m_i2c_instance, &i2c_config);
    APP_ERROR_CHECK(err_code);

    // Uncomment for bma280 calibration.
    /*
    err_code = bma280_calibrate(&m_i2c_instance, BMA280_I2C_ADDRESS);
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(500);
    */

    const bma280_init_t bma_config = 
    {
        .ascale         = AFS_2G,
        .BW             = BW_7_81Hz,
        .power_mode     = lowPower_Mode,
        .sleep_dur      = sleep_500ms
    };

    // Now we can init bma280 sensor with our i2c instance.
    err_code = bma280_init(&bma_config, &m_i2c_instance, BMA280_I2C_ADDRESS);
    APP_ERROR_CHECK(err_code);

    // Set bma280 working mode.
    bma280_set_interrupt_detect_mode_slow(&m_i2c_instance, BMA280_I2C_ADDRESS);
    bma280_handle();
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
__STATIC_INLINE void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false && i2s_was_playing_finished() == true)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    gpio_init();
    power_management_init();
    ble_init();
    accelerometer_init();
    snd_init();

    // Start execution.
    NRF_LOG_INFO("Program started.");

    // Enter main loop.
    while(true) 
    {
        bma280_handle();
        i2s_handle();

        idle_state_handle();
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
__STATIC_INLINE void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
__STATIC_INLINE void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
__STATIC_INLINE void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief bma280 interrupt handler.
 */
__STATIC_INLINE void bma280_interrupt_handler()
{
    m_bma280_int_detected = true;
}

/**
 * @brief Function for configuring: bma280_INT2_PIN pin for input
 * and configures GPIOTE to give an interrupt on pin change.
 */
__STATIC_INLINE void gpio_init(void)
{
    ret_code_t err_code;

    if(!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    const nrf_drv_gpiote_in_config_t in_config = {
        .pull            = NRF_GPIO_PIN_PULLUP,
        .sense           = GPIOTE_CONFIG_POLARITY_Toggle,
        .hi_accuracy     = true,
        .is_watcher      = false,
        .skip_gpio_setup = false
    };

    err_code = nrf_drv_gpiote_in_init(BMA280_INT2_PIN, &in_config, (nrfx_gpiote_evt_handler_t)bma280_interrupt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BMA280_INT2_PIN, true);
}


/**
 * @}
 */