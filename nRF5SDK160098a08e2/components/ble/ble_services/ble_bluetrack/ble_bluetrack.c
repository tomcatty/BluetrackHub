/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_BLUETRACK)
#include "ble_bluetrack.h"
#include "ble_srv_common.h"


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bluetrack  BlueTrack Service structure.
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 */
static void on_write(ble_bluetrack_t * p_bluetrack, ble_evt_t const * p_ble_evt)
{
    // Get write event.
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    // Only call the Address Write handler if the Address Characteristic was written to, the write was of the correct length, and a handler exists.
    if ((p_evt_write->handle == p_bluetrack->address_char_handles.value_handle) &&
        (p_evt_write->len == ADDRESS_CHAR_SIZE) &&
        (p_bluetrack->address_write_handler != NULL))
    {
        p_bluetrack->address_write_handler(p_bluetrack, p_evt_write->data[0]);
    }

    // Only call the DCC Command Write handler if the DCC Command Characteristic was written to, the write was of the correct length, and a handler exists.
    if ((p_evt_write->handle == p_bluetrack->dcc_command_char_handles.value_handle) &&
        (p_evt_write->len == DCC_COMMAND_CHAR_SIZE) &&
        (p_bluetrack->dcc_command_write_handler != NULL))
    {
        p_bluetrack->dcc_command_write_handler(p_bluetrack, p_evt_write->data[0], p_evt_write->data[1], p_evt_write->data[2], p_evt_write->data[3], p_evt_write->data[4], p_evt_write->data[5]);
    }

    // Only call the Programming Track Select Write handler if the Programming Track Select Characteristic was written to, the write was of the correct length, and a handler exists.
    if ((p_evt_write->handle == p_bluetrack->programming_track_select_char_handles.value_handle) &&
        (p_evt_write->len == PROGRAMMING_TRACK_SELECT_CHAR_SIZE) &&
        (p_bluetrack->programming_track_select_write_handler != NULL))
    {
        p_bluetrack->programming_track_select_write_handler(p_bluetrack, p_evt_write->data[0]);
    }

    // Only call the Stop Write handler if the Stop Characteristic was written to, the write was of the correct length, and a handler exists.
    if ((p_evt_write->handle == p_bluetrack->stop_char_handles.value_handle) &&
        (p_evt_write->len == STOP_CHAR_SIZE) &&
        (p_bluetrack->stop_write_handler != NULL))
    {
        p_bluetrack->stop_write_handler(p_bluetrack, p_evt_write->data[0]);
    }
    
    // Only call the Service Command Write handler if the Service Command Characteristic was written to, the write was of the correct length, and a handler exists.
    if ((p_evt_write->handle == p_bluetrack->service_command_char_handles.value_handle) &&
        (p_evt_write->len == SERVICE_COMMAND_CHAR_SIZE) &&
        (p_bluetrack->service_command_write_handler != NULL))
    {
        p_bluetrack->service_command_write_handler(p_bluetrack, p_evt_write->data[0], p_evt_write->data[1], p_evt_write->data[2], p_evt_write->data[3], p_evt_write->data[4]);
    }
}


void ble_bluetrack_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    // Cast p_context as ble_bluetrack_t, doesn't seem safe...
    ble_bluetrack_t * p_bluetrack = (ble_bluetrack_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_bluetrack, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Address Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t address_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_ADDRESS_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = ADDRESS_CHAR_SIZE;
    add_char_params.init_len          = ADDRESS_CHAR_SIZE;
    add_char_params.p_init_value      = NULL;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 0;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_NO_ACCESS;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->address_char_handles);
}


/**@brief Function for adding the DCC Command Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t dcc_command_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_DCC_COMMAND_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = DCC_COMMAND_CHAR_SIZE;
    add_char_params.init_len          = DCC_COMMAND_CHAR_SIZE;
    add_char_params.p_init_value      = NULL;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 0;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_NO_ACCESS;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->dcc_command_char_handles);
}


/**@brief Function for adding the Programming Track Select Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t programming_track_select_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;
    uint8_t               initial_value = 0x00; // Zero out characteristic initially

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_PROGRAMMING_TRACK_SELECT_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = PROGRAMMING_TRACK_SELECT_CHAR_SIZE;
    add_char_params.init_len          = PROGRAMMING_TRACK_SELECT_CHAR_SIZE;
    add_char_params.p_init_value      = &initial_value;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->programming_track_select_char_handles);
}


/**@brief Function for adding the Stop Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t stop_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;
    uint8_t               initial_value = 0x01; // Set characteristic initially

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_STOP_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = STOP_CHAR_SIZE;
    add_char_params.init_len          = STOP_CHAR_SIZE;
    add_char_params.p_init_value      = &initial_value;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->stop_char_handles);
}


/**@brief Function for adding the Response Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t response_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_RESPONSE_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = RESPONSE_CHAR_SIZE;
    add_char_params.init_len          = RESPONSE_CHAR_SIZE;
    add_char_params.p_init_value      = NULL;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 0;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->response_char_handles);
}


/**@brief Function for adding the Service Command Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t service_command_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_SERVICE_COMMAND_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = SERVICE_COMMAND_CHAR_SIZE;
    add_char_params.init_len          = SERVICE_COMMAND_CHAR_SIZE;
    add_char_params.p_init_value      = NULL;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.char_props.notify = 0;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_NO_ACCESS;
    
    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->service_command_char_handles);
}


/**@brief Function for adding the Error Characteristic.
 *
 * @param[in]   p_bluetrack       BlueTrack Service structure.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t error_char_add(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    ble_add_char_params_t add_char_params;
    uint8_t               initial_value = 0x00; // Zero out characteristic initially

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLUETRACK_UUID_ERROR_CHAR;
    add_char_params.uuid_type         = p_bluetrack->uuid_type;
    add_char_params.max_len           = ERROR_CHAR_SIZE;
    add_char_params.init_len          = ERROR_CHAR_SIZE;
    add_char_params.p_init_value      = &initial_value;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 0;
    add_char_params.char_props.notify = 1;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_bluetrack->service_handle,
                              &add_char_params,
                              &p_bluetrack->error_char_handles);
}


uint32_t ble_bluetrack_init(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_bluetrack->address_write_handler                  = p_bluetrack_init->address_write_handler;
    p_bluetrack->dcc_command_write_handler              = p_bluetrack_init->dcc_command_write_handler;
    p_bluetrack->programming_track_select_write_handler = p_bluetrack_init->programming_track_select_write_handler;
    p_bluetrack->stop_write_handler                     = p_bluetrack_init->stop_write_handler;
    p_bluetrack->service_command_write_handler          = p_bluetrack_init->service_command_write_handler;

    // Add base UUID to ble stack list.
    ble_uuid128_t base_uuid = BLUETRACK_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_bluetrack->uuid_type);
    VERIFY_SUCCESS(err_code);

    // Add BlueTrack service.
    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bluetrack->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add Address Characteristic.
    err_code = address_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    // Add DCC Command Characteristic
    err_code = dcc_command_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    // Add Programming Track Select Characteristic
    err_code = programming_track_select_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    // Add Stop Characteristic
    err_code = stop_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    // Add Response Characteristic
    err_code = response_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);
    
    // Add Service Command Characteristic
    err_code = service_command_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    // Add Error Characteristic
    err_code = error_char_add(p_bluetrack, p_bluetrack_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t ble_bluetrack_response_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack, uint8_t response_valid, uint8_t response_value)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = RESPONSE_CHAR_SIZE;
    uint8_t data[RESPONSE_CHAR_SIZE];
    
    data[0] = response_valid;
    data[1] = response_value;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->response_char_handles.value_handle;
    params.p_data = data;
    params.p_len = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

uint32_t ble_bluetrack_programming_track_select_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack)
{
    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->programming_track_select_char_handles.value_handle;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

uint32_t ble_bluetrack_stop_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack)
{
    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->stop_char_handles.value_handle;

    return sd_ble_gatts_hvx(conn_handle, &params);
}

uint32_t ble_bluetrack_error_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack, uint8_t error_code)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = ERROR_CHAR_SIZE;
    uint8_t data[ERROR_CHAR_SIZE];
    
    data[0] = error_code;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->error_char_handles.value_handle;
    params.p_data = data;
    params.p_len = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}
#endif // NRF_MODULE_ENABLED(BLE_BLUETRACK)