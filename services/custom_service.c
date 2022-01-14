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

#include <string.h>

#include "nrf_log.h"
#include "custom_service.h"

static const uint8_t AudioCharName[] = "Audio";
static const uint8_t AccelCharName[] = "Accelerometer";

/**@brief Function for handling the Connect event.
 *
 * @param[in]   ble_custom_service  Custom service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void on_connect(ble_custom_service_t * p_custom_service, ble_evt_t const * p_ble_evt)
{
    p_custom_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   ble_custom_service  Custom service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void on_disconnect(ble_custom_service_t * p_custom_service, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in] p_custom_service   Custom Service structure.
 * @param[in] p_ble_evt          Event received from the BLE stack.
 */
static void on_write(ble_custom_service_t * p_custom_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_custom_service->audio_char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_custom_service->audio_write_handler != NULL))
    {
        p_custom_service->audio_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_custom_service, p_evt_write->data[0]);
    }
}

/**@brief Template function for adding new characteristic.
 *
 * @param[in] service_handle   Pointer to BLE service handle.
 * @param[in] char_handle      Pointer to characteristic handle.
 * @param[in] char_name        Characterictic's name.
 * @param[in] uuid             UUID for characteristic.
 * @param[in] attr_len         Characteristic's attribute length in bytes.
 * @param[in] is_readable      Flag if characteristic is readable.
 * @param[in] is_writeable     Flag if characteristic is writeable.
 */
static uint32_t char_add(const uint16_t service_handle, ble_gatts_char_handles_t * char_handle, const uint8_t * char_name, const uint16_t uuid, const uint8_t attr_len, const bool is_readable, const bool is_writeable)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t          ble_uuid;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));
    memset(&attr_md, 0, sizeof(attr_md));
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    char_md.char_props.read          = is_readable;
    char_md.char_props.write         = is_writeable;
    char_md.p_char_user_desc         = char_name;
    char_md.char_user_desc_size      = strlen(char_name);
    char_md.char_user_desc_max_size  = strlen(char_name);
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    ble_uuid.uuid = uuid;

    // Set permissions on the Characteristic value
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);

    // Attribute Metadata settings
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    // Attribute Value settings
    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = attr_len;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = attr_len;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(service_handle, &char_md,
                                           &attr_char_value,
                                           char_handle);
}


uint32_t ble_custom_service_init(ble_custom_service_t * p_custom_service, const ble_custom_service_init_t * p_custom_service_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_custom_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Initialize service structure.
    p_custom_service->audio_write_handler = p_custom_service_init->audio_write_handler;

    // Add service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_custom_service->uuid_type);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Set up the UUID for the service (base + service-specific)
    ble_uuid.type = p_custom_service->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Set up and add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_custom_service->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    // Add the different characteristics in the service
    err_code = char_add(p_custom_service->service_handle, 
                        &p_custom_service->audio_char_handles, 
                        AudioCharName, 
                        CUSTOM_AUDIO_CHAR_UUID, 1, false, true);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    err_code = char_add(p_custom_service->service_handle, 
                        &p_custom_service->accel_char_handles, 
                        AccelCharName, 
                        CUSTOM_ACCEL_CHAR_UUID, 6, true, false);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_custom_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_custom_service_t * p_custom_service = (ble_custom_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_custom_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_custom_service, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_custom_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_custom_service_on_accel_int(ble_custom_service_t * p_custom_service, BMA280_accel_values_t * data)
{
    ble_gatts_value_t gatts_value; 

    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len = sizeof(BMA280_accel_values_t);
    gatts_value.offset = 0;
    gatts_value.p_value = (uint8_t*) data;
    return sd_ble_gatts_value_set(p_custom_service->conn_handle, 
                                  p_custom_service->accel_char_handles.value_handle, 
                                  &gatts_value);
}