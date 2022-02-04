#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include "ble_advertising.h"
#include "ble_service.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_MANAGER_DEF(_name, _count)       \                                  /**< Definition for our BLE Manager. */
BLE_SERVICE_DEF(_name, _count)

#ifndef DEVICE_BLE_NAME
#define DEVICE_BLE_NAME                 "BLE_device"                            /**< Name of device. Will be included in the advertising data. */
#endif

#ifndef MANUFACTURER_NAME
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#endif

#ifndef APP_ADV_INTERVAL
#define APP_ADV_INTERVAL                MSEC_TO_UNITS(1000, UNIT_0_625_MS)      /**< The advertising interval for non-connectable advertisement. This value can vary between 100ms to 10.24s). */
#endif

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

/**@brief Function for init BLE Manager.
 * Run ble_stack, gap_params, gatt, advertising, qwr inits.
 *
 * @param[in] ble_device_name  BLE device name (string).
 *
 * @return                NRF_SUCCESS on successful operation, otherwise an error code.
 */
ret_code_t ble_manager_init(const uint8_t const * ble_device_name);

/**@brief Function for running BLE Manager finalize init.
 * Run connection parameters and peer manager modules.
 *
 * @return                NRF_SUCCESS on successful operation, otherwise an error code.
 */
ret_code_t ble_manager_init_finalize();

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
ret_code_t ble_manager_add_service(ble_service_t * p_service, const ble_service_init_t const * p_service_init);

/**@brief Function for adding new characteristic.
 *
 * @param[in] p_service        Pointer to BLE service handle.
 * @param[in] p_char_init      Pointer to characteristic's init structure.
 * @param[out] p_char_handles  Pointer to characteristic handle, if need to get.
 *
 * @retval ::NRF_SUCCESS Successfully added a characteristic.
 * @retval ::NRF_ERROR_INVALID_ADDR Invalid pointer supplied.
 * @retval ::NRF_ERROR_INVALID_PARAM Invalid parameter(s) supplied, service handle, Vendor Specific UUIDs, lengths, and permissions need to adhere to the constraints.
 * @retval ::NRF_ERROR_INVALID_STATE Invalid state to perform operation, a service context is required.
 * @retval ::NRF_ERROR_FORBIDDEN Forbidden value supplied, certain UUIDs are reserved for the stack.
 * @retval ::NRF_ERROR_NO_MEM Not enough memory to complete operation.
 * @retval ::NRF_ERROR_DATA_SIZE Invalid data size(s) supplied, attribute lengths are restricted by @ref BLE_GATTS_ATTR_LENS_MAX.
 */
ret_code_t ble_manager_add_characteristic(ble_service_t * p_service, 
                                          const ble_characteristic_init_t const * p_char_init, 
                                          ble_gatts_char_handles_t * p_char_handle);

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
ret_code_t ble_manager_update_char_value(const ble_service_t const * p_service, 
                                         const ble_gatts_char_handles_t const * p_char_handles, 
                                         const void const* data, 
                                         const uint16_t data_len);

/**@brief Function for starting advertising.
 *
 * @return                NRF_SUCCESS on successful operation, otherwise an error code.
 */
ret_code_t ble_manager_advertising_start(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_MANAGER_H */
