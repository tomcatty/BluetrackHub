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

#include "ble_bluetrack.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bluetrack  BlueTrack Service structure.
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 */
static void on_connect(ble_bluetrack_t * p_bluetrack, ble_evt_t * p_ble_evt)
{
    p_bluetrack->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bluetrack  BlueTrack Service structure.
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 */
static void on_disconnect(ble_bluetrack_t * p_bluetrack, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bluetrack->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bluetrack  BlueTrack Service structure.
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 */
static void on_write(ble_bluetrack_t * p_bluetrack, ble_evt_t * p_ble_evt)
{
    // Get write event.
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

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


void ble_bluetrack_on_ble_evt(ble_bluetrack_t * p_bluetrack, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bluetrack, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_bluetrack, p_ble_evt);
            break;

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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_ADDRESS_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = ADDRESS_CHAR_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = ADDRESS_CHAR_SIZE;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                               &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_DCC_COMMAND_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = DCC_COMMAND_CHAR_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = DCC_COMMAND_CHAR_SIZE;
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                               &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_value = 0x00; // Zero out characteristic initially

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_PROGRAMMING_TRACK_SELECT_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = PROGRAMMING_TRACK_SELECT_CHAR_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = PROGRAMMING_TRACK_SELECT_CHAR_SIZE;
    attr_char_value.p_value   = &initial_value;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                           &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_value = 0x01; // Set characteristic initially

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_STOP_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = STOP_CHAR_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = STOP_CHAR_SIZE;
    attr_char_value.p_value   = &initial_value;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                           &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_RESPONSE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = RESPONSE_CHAR_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = RESPONSE_CHAR_SIZE;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                           &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_SERVICE_COMMAND_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = SERVICE_COMMAND_CHAR_SIZE;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = SERVICE_COMMAND_CHAR_SIZE;
    attr_char_value.p_value   = NULL;
    
    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                           &attr_char_value,
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
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_value = 0x00; // Set characteristic initially

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_ERROR_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = ERROR_CHAR_SIZE;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = ERROR_CHAR_SIZE;
    attr_char_value.p_value      = &initial_value;

    return sd_ble_gatts_characteristic_add(p_bluetrack->service_handle, &char_md,
                                           &attr_char_value,
                                           &p_bluetrack->error_char_handles);
}


uint32_t ble_bluetrack_init(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_bluetrack->conn_handle                            = BLE_CONN_HANDLE_INVALID;
    p_bluetrack->address_write_handler                  = p_bluetrack_init->address_write_handler;
    p_bluetrack->dcc_command_write_handler              = p_bluetrack_init->dcc_command_write_handler;
    p_bluetrack->programming_track_select_write_handler = p_bluetrack_init->programming_track_select_write_handler;
    p_bluetrack->stop_write_handler                     = p_bluetrack_init->stop_write_handler;
    p_bluetrack->service_command_write_handler          = p_bluetrack_init->service_command_write_handler;

    // Add base UUID to ble stack list.
    ble_uuid128_t base_uuid = BLUETRACK_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_bluetrack->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add BlueTrack service.
    ble_uuid.type = p_bluetrack->uuid_type;
    ble_uuid.uuid = BLUETRACK_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bluetrack->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Address Characteristic.
    err_code = address_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add DCC Command Characteristic
    err_code = dcc_command_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Programming Track Select Characteristic
    err_code = programming_track_select_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Stop Characteristic
    err_code = stop_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Response Characteristic
    err_code = response_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add Service Command Characteristic
    err_code = service_command_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add Error Characteristic
    err_code = error_char_add(p_bluetrack, p_bluetrack_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_bluetrack_response_update(ble_bluetrack_t * p_bluetrack, uint8_t response_valid, uint8_t response_value)
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

    return sd_ble_gatts_hvx(p_bluetrack->conn_handle, &params);
}

uint32_t ble_bluetrack_programming_track_select_update(ble_bluetrack_t * p_bluetrack)
{
    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->programming_track_select_char_handles.value_handle;

    return sd_ble_gatts_hvx(p_bluetrack->conn_handle, &params);
}

uint32_t ble_bluetrack_stop_update(ble_bluetrack_t * p_bluetrack)
{
    ble_gatts_hvx_params_t params;

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_bluetrack->stop_char_handles.value_handle;

    return sd_ble_gatts_hvx(p_bluetrack->conn_handle, &params);
}

uint32_t ble_bluetrack_error_update(ble_bluetrack_t * p_bluetrack, uint8_t error_code)
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

    return sd_ble_gatts_hvx(p_bluetrack->conn_handle, &params);
}
