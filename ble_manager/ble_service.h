#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SERVICE_DEF(_name,_count)                                                                   \
static ble_service_t _name[_count];                                                                     \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                     \
                     BLE_SERVICE_BLE_OBSERVER_PRIO,                                                     \
                     ble_service_on_ble_evt, &_name)


#ifndef BLE_MAXIMUM_CHARS_IN_SERVICE
#error "Not defined BLE_MAXIMUM_CHARS_IN_SERVICE in *_config.h"
#endif

#define BLE_SERVICE_BLE_OBSERVER_PRIO 2


/**< Forward declaration of the ble_characteristic_t type. */
typedef struct ble_characteristic_s ble_characteristic_t;

/**< Event handler to be called when the characteristic is written. */
typedef void (*ble_service_write_handler_t) (const ble_characteristic_t const*, const uint8_t*, const uint16_t);

/**< Ble characteristic structure. */
typedef struct ble_characteristic_s
{
    ble_gatts_char_handles_t    char_handle;
    const uint8_t *             char_name;
    const uint16_t              char_uuid;
    bool                        is_readable;
    bool                        is_writeable;
    uint8_t                     attr_len;
    ble_service_write_handler_t write_handler;
} ble_characteristic_t;

/** @brief Characteristic init structure. This structure contains all options and data needed for
 *        initialization of the characteristic.
 */
typedef struct ble_characteristic_init_s
{
    const uint8_t *             char_name;
    const uint16_t              char_uuid;
    bool                        is_readable;
    bool                        is_writeable;
    uint8_t                     attr_len;
    ble_service_write_handler_t write_handler;
} ble_characteristic_init_t;

/**@brief Service structure.
 *        This contains various status information
 *        for the service.
 */
typedef struct ble_service_s
{
    uint16_t                    conn_handle;
    uint16_t                    service_handle;
    uint8_t                     uuid_type;
    uint8_t                     chars_count;
    ble_characteristic_t        chars[BLE_MAXIMUM_CHARS_IN_SERVICE]; // Array of Characteristics

} ble_service_t;

/**@brief Service init structure.
 */
typedef struct ble_service_init_s
{
    ble_uuid128_t               base_uuid;
    uint16_t                    uuid;

} ble_service_init_t;

// Function Declarations

/**@brief Function for adding new characteristic.
 *
 * @param[in] p_service        Pointer to BLE service handle.
 * @param[in] p_char_init      Pointer to characteristic's init structure.
 * @param[out] p_char_handles  Pointer to characteristic handle, if need to get.
 */
ret_code_t ble_service_add_characteristic(ble_service_t * p_service, 
                                          const ble_characteristic_init_t const * p_char_init, 
                                          ble_gatts_char_handles_t * p_char_handles);

/**@brief Function for adding new Service.
 *
 * @param[out]  p_service      Service structure. This structure will have to be supplied by
 *                             the application. It will be initialized by this function, and will later
 *                             be used to identify this particular service instance.
 * @param[in] p_service_init   Pointer to Service init structure.
 *
 * @retval ::NRF_SUCCESS On successful result.
 * @retval ::NRF_ERROR_INVALID_ADDR If p_vs_uuid or p_uuid_type is NULL or invalid.
 * @retval ::NRF_ERROR_NO_MEM If there are no more free slots for VS UUI.
 * @retval ::NRF_ERROR_INVALID_ADDR Invalid pointer supplied.
 * @retval ::NRF_ERROR_INVALID_PARAM Invalid parameter(s) supplied, Vendor Specific UUIDs need to be present in the table.
 * @retval ::NRF_ERROR_FORBIDDEN Forbidden value supplied, certain UUIDs are reserved for the stack.
 * @retval ::NRF_ERROR_NO_MEM Not enough memory to complete operation.
 */
ret_code_t ble_service_create(ble_service_t * p_service, const ble_service_init_t const * p_service_init);

/**@brief Function for writing new value in existing characteristic.
 *
 * @param[in]  p_service       To identify particular service instance.
 * @param[in]  p_char_handles  Pointer to characteristic handle.
 * @param[in]  data            Pointer to buffer's data.
 * @param[in]  data_len        Size of buffer.
 *
 * @retval ::NRF_SUCCESS Successfully set the value of the attribute.
 * @retval ::NRF_ERROR_INVALID_ADDR Invalid pointer supplied.
 * @retval ::NRF_ERROR_INVALID_PARAM Invalid parameter(s) supplied.
 * @retval ::NRF_ERROR_NOT_FOUND Attribute not found.
 * @retval ::NRF_ERROR_FORBIDDEN Forbidden handle supplied, certain attributes are not modifiable by the application.
 * @retval ::NRF_ERROR_DATA_SIZE Invalid data size(s) supplied, attribute lengths are restricted by @ref BLE_GATTS_ATTR_LENS_MAX.
 * @retval ::BLE_ERROR_INVALID_CONN_HANDLE Invalid connection handle supplied on a system attribute.
 */
ret_code_t ble_service_update_char_value(const ble_service_t const * p_service, 
                                         const ble_gatts_char_handles_t const * p_char_handles, 
                                         const void const * data, 
                                         const uint8_t data_len);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Custom Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Custom Service structure.
 */
void ble_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#ifdef __cplusplus
}
#endif

#endif /* BLE_SERVICE_H */