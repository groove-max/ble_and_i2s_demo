#include <string.h>

#include "nrf_log.h"
#include "boards.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#include "ble_service.h"


ret_code_t ble_service_add_characteristic(ble_service_t * p_service, 
                                          const ble_characteristic_init_t const * p_char_init, 
                                          ble_gatts_char_handles_t * p_char_handle)
{
    ASSERT(p_service);
    ASSERT(p_char_init);

    if(p_service->chars_count >= BLE_MAXIMUM_CHARS_IN_SERVICE) {
        return NRF_ERROR_INVALID_LENGTH;
    }

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

    char_md.char_props.read          = p_char_init->is_readable;
    char_md.char_props.write         = p_char_init->is_writeable;
    char_md.p_char_user_desc         = p_char_init->char_name;
    char_md.char_user_desc_size      = strlen(p_char_init->char_name);
    char_md.char_user_desc_max_size  = strlen(p_char_init->char_name);
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = &cccd_md;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
    ble_uuid.uuid = p_char_init->char_uuid;

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
    attr_char_value.init_len     = p_char_init->attr_len;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = p_char_init->attr_len;
    attr_char_value.p_value      = NULL;

    ret_code_t err_code;
    ble_gatts_char_handles_t p_handle;
    err_code = sd_ble_gatts_characteristic_add(p_service->service_handle, &char_md, &attr_char_value, &p_handle);
    CHECK_ERROR_RETURN(err_code);
    p_service->chars[p_service->chars_count].char_handle = p_handle;
    p_service->chars[p_service->chars_count].write_handler = p_char_init->write_handler;
    if(p_char_handle != NULL) {
        *p_char_handle = p_handle;
    }

    p_service->chars_count++;
    return NRF_SUCCESS;
}


ret_code_t ble_service_create(ble_service_t * p_service, const ble_service_init_t const * p_service_init)
{
    ret_code_t  err_code;
    ble_uuid_t ble_uuid;

    p_service->chars_count = 0;

    memset(&p_service->chars[0], 0, sizeof(ble_characteristic_t) * BLE_MAXIMUM_CHARS_IN_SERVICE);

    // Initialize service structure
    p_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service UUID
    //ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&p_service_init->base_uuid, &p_service->uuid_type);
    CHECK_ERROR_RETURN(err_code);

    // Set up the UUID for the service (base + service-specific)
    ble_uuid.type = p_service->uuid_type;
    ble_uuid.uuid = p_service_init->uuid;

    // Set up and add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_service->service_handle);
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}


ret_code_t ble_service_update_char_value(const ble_service_t const * p_service, 
                                         const ble_gatts_char_handles_t const * p_char_handles, 
                                         const void const * data, 
                                         const uint8_t data_len)
{
    ASSERT(p_service);
    ASSERT(p_char_handles);
    ASSERT(data);
    ASSERT(data_len);

    ret_code_t        err_code;
    ble_gatts_value_t gatts_value; 

    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len = data_len;
    gatts_value.offset = 0;
    gatts_value.p_value = (uint8_t*) data;
    err_code = sd_ble_gatts_value_set(p_service->conn_handle, p_char_handles->value_handle, &gatts_value);
    CHECK_ERROR_RETURN(err_code);

    return NRF_SUCCESS;
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   ble_service         Custom service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void ble_service_on_connect(ble_service_t * p_ble_service, ble_evt_t const * p_ble_evt)
{
    ASSERT(p_ble_service);
    ASSERT(p_ble_evt);

    p_ble_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   ble_service         Custom service structure.
 * @param[in]   p_ble_evt           Event received from the BLE stack.
 */
static void ble_service_on_disconnect(ble_service_t * p_ble_service, ble_evt_t const * p_ble_evt)
{
    ASSERT(p_ble_service);
    ASSERT(p_ble_evt);

    UNUSED_PARAMETER(p_ble_evt);
    p_ble_service->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in] p_service   Custom Service structure.
 * @param[in] p_ble_evt          Event received from the BLE stack.
 */
static void ble_service_on_write(ble_service_t * p_service, ble_evt_t const * p_ble_evt)
{
    ASSERT(p_service);
    ASSERT(p_ble_evt);

    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    ble_characteristic_t * needed_char = NULL;
    ble_characteristic_t * active_char = NULL;

    for(uint8_t i=0; i < p_service->chars_count; i++) {
        active_char = &p_service->chars[i];
        if(active_char->char_handle.value_handle == p_evt_write->handle) {
            needed_char = active_char;
            break;
        }
    }

    if(needed_char != NULL && needed_char->write_handler != NULL) {
        needed_char->write_handler(needed_char, &p_evt_write->data[0], p_evt_write->len);
    }
}


void ble_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ASSERT(p_ble_evt);
    ASSERT(p_context);

    ble_service_t * p_ble_service = (ble_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            ble_service_on_connect(p_ble_service, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            ble_service_on_write(p_ble_service, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_service_on_disconnect(p_ble_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}
