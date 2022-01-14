/*
 * The MIT License (MIT)
 * Copyright (c) 2018 Novel Bits
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#ifndef CUSTOM_SERVICE_H
#define CUSTOM_SERVICE_H

#include <stdint.h>
#include "boards.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#include "../sensors/BMA280.h"

/**@brief   Macro for defining a ble_service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */

#define BLE_CUSTOM_SERVICE_BLE_OBSERVER_PRIO 2

#define BLE_CUSTOM_SERVICE_DEF(_name)                                                                          \
static ble_custom_service_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                            \
                     BLE_CUSTOM_SERVICE_BLE_OBSERVER_PRIO,                                                     \
                     ble_custom_service_on_ble_evt, &_name)

// The bytes are stored in little-endian format, meaning the
// Least Significant Byte is stored first
// (reversed from the order they're displayed as)

//6ac131d4-6985-4b39-80e7-1740b2426d2d
#define CUSTOM_SERVICE_UUID_BASE         {0x2D, 0x6D, 0x42, 0xB2, 0x40, 0x17, 0xE7, 0x80, \
                                          0x39, 0x4B, 0x85, 0x69, 0xD4, 0x31, 0xC1, 0x6A}

#define CUSTOM_SERVICE_UUID               0x1400
#define CUSTOM_AUDIO_CHAR_UUID            0x1401
#define CUSTOM_ACCEL_CHAR_UUID            0x1402

// Forward declaration of the custom_service_t type.
typedef struct ble_custom_service_s ble_custom_service_t;

typedef void (*ble_custom_service_audio_write_handler_t) (uint16_t conn_handle, ble_custom_service_t * p_custom_service, uint8_t new_state);

/** @brief Custom Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_custom_service_audio_write_handler_t audio_write_handler; /**< Event handler to be called when the Custom Characteristic is written. */
} ble_custom_service_init_t;

/**@brief Custom Service structure.
 *        This contains various status information
 *        for the service.
 */
typedef struct ble_custom_service_s
{
    uint16_t                                 conn_handle;
    uint16_t                                 service_handle;
    uint8_t                                  uuid_type;
    ble_gatts_char_handles_t                 audio_char_handles;
    ble_gatts_char_handles_t                 accel_char_handles;
    ble_custom_service_audio_write_handler_t audio_write_handler;

} ble_custom_service_t;

// Function Declarations

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  ble_custom_service  Custom Service structure. This structure will have to be supplied by
 *                                  the application. It will be initialized by this function, and will later
 *                                  be used to identify this particular service instance.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_custom_service_init(ble_custom_service_t * p_custom_service, const ble_custom_service_init_t * p_custom_service_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Custom Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Custom Service structure.
 */
void ble_custom_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for write accelerometer's data to the Accelerometer characteristic.
 *
 * @param[in]  p_custom_service   To identify particular service instance.
 * @param[in]  data               Values to be written.
 *
 * @return      NRF_SUCCESS on successful writing of values, otherwise an error code.
 */
uint32_t ble_custom_service_on_accel_int(ble_custom_service_t * p_custom_service, BMA280_accel_values_t * data);

#endif /* CUSTOM_SERVICE_H */