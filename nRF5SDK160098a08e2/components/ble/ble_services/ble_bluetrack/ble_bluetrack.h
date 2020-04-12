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

/** @file
 *
 * @defgroup ble_cust_srv_bluetrack BlueTrack Service
 * @{
 * @ingroup ble_cust_srv
 * @brief BlueTrack Service module.
 *
 * @details This module implements the BlueTrack Service.
 *          During initialization it adds the BlueTrack Service, Address Characteristic, DCC Command Characteristic,
 *          Programming Track Select Characteristic, Stop Characteristic, 
 *          and Acknowledge Characteristic to the BLE stack database.
 *
 *          If an Address Write handler is supplied by the application, the BlueTrack Service will
 *          call this handler when the Address Characteristic is written to.
 *
 *          If a DCC Command Write handler is supplied by the application, the BlueTrack Service will
 *          call this handler when the DCC Command Characteristic is written to.
 *
 *          If a Programming Track Select Write handler is supplied by the application, the BlueTrack Service will
 *          call this handler when the Programming Track Select Characteristic is written to.
 *
 *          If a Stop Write handler is supplied by the application, the BlueTrack Service will
 *          call this handler when the Stop Characteristic is written to.
 *
 * @note The application must propagate BLE stack events to the BlueTrack Service module by calling
 *       ble_bluetrack_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_BLUETRACK_H__
#define BLE_BLUETRACK_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/**@brief   Macro for defining a ble_bluetrack instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_BLUETRACK_DEF(_name)                                                                          \
static ble_bluetrack_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                       \
                     BLE_BLUETRACK_BLE_OBSERVER_PRIO,                                                     \
                     ble_bluetrack_on_ble_evt, &_name)

// Generated using nRFgo Studio
#define BLUETRACK_UUID_BASE {0x21, 0xAE, 0x66, 0x08, 0x3B, 0x47, 0x26, 0xE6, 0x5D, 0x68, 0x68, 0xC7, 0x00, 0x00, 0x51, 0x30}
#define BLUETRACK_UUID_SERVICE 0x1523
#define BLUETRACK_UUID_ADDRESS_CHAR 0x1524
#define BLUETRACK_UUID_DCC_COMMAND_CHAR 0x1525
#define BLUETRACK_UUID_PROGRAMMING_TRACK_SELECT_CHAR 0x1526
#define BLUETRACK_UUID_STOP_CHAR 0x1527
#define BLUETRACK_UUID_RESPONSE_CHAR 0x1528
#define BLUETRACK_UUID_SERVICE_COMMAND_CHAR 0x1529
#define BLUETRACK_UUID_ERROR_CHAR 0x1530

#define ADDRESS_CHAR_SIZE 1
#define DCC_COMMAND_CHAR_SIZE 6
#define PROGRAMMING_TRACK_SELECT_CHAR_SIZE 1
#define STOP_CHAR_SIZE 1
#define RESPONSE_CHAR_SIZE 2
#define SERVICE_COMMAND_CHAR_SIZE 5
#define ERROR_CHAR_SIZE 1

#define ERROR_CODE_MAX_TRAIN_EXCEEDED 1
#define ERROR_CODE_OVERTEMPERATURE 2
#define ERROR_CODE_OVERCURRENT 3

// Forward declaration of the ble_bluetrack_t type. 
typedef struct ble_bluetrack_s ble_bluetrack_t;


/**@brief BlueTrack Service event handler type. */
typedef void (*ble_bluetrack_address_write_handler_t) (ble_bluetrack_t * p_bluetrack, uint8_t address);
typedef void (*ble_bluetrack_dcc_command_write_handler_t) (ble_bluetrack_t * p_bluetrack, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t len);
typedef void (*ble_bluetrack_programming_track_select_write_handler_t) (ble_bluetrack_t * p_bluetrack, uint8_t select);
typedef void (*ble_bluetrack_stop_write_handler_t) (ble_bluetrack_t * p_bluetrack, uint8_t stop);
typedef void (*ble_bluetrack_service_command_write_handler_t) (ble_bluetrack_t * p_bluetrack, uint8_t function_local, uint8_t mode_local, uint8_t CV_or_reg_upper_local, uint8_t CV_or_reg_lower_local, uint8_t value_local);


/**@brief Bluetrack Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_bluetrack_address_write_handler_t                  address_write_handler;                  /**< Address Write handler to be called for handling writes to the Address Characteristic in the application. */
    ble_bluetrack_dcc_command_write_handler_t              dcc_command_write_handler;              /**< DCC Command Write handler to be called for handling writes to the DCC Command Characteristic in the application. */
    ble_bluetrack_programming_track_select_write_handler_t programming_track_select_write_handler; /**< Programming Track Select Write handler to be called for handling writes to the Programming Track Select Characteristic in the application. */
    ble_bluetrack_stop_write_handler_t                     stop_write_handler;                     /**< Stop Write handler to be called for handling writes to the Stop Characteristic in the application. */
    ble_bluetrack_service_command_write_handler_t          service_command_write_handler;          /**< Service Command Write handler to be called for handling writes to the Service Command Characteristic in the application. */
} ble_bluetrack_init_t;


/**@brief BlueTrack Service structure. This contains various status information for the service. */
typedef struct ble_bluetrack_s
{
    uint16_t                                               service_handle;                         /**< Handle of BlueTrack Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t                               address_char_handles;                   /**< Handles related to the Address Characteristic. */
    ble_gatts_char_handles_t                               dcc_command_char_handles;               /**< Handles related to the DCC Command Characteristic. */
    ble_gatts_char_handles_t                               programming_track_select_char_handles;  /**< Handles related to the Programming Track Select Characteristic */
    ble_gatts_char_handles_t                               stop_char_handles;                      /**< Handles related to the Stop Characteristic */
    ble_gatts_char_handles_t                               response_char_handles;                  /**< Handles related to the Response Characteristic */
    ble_gatts_char_handles_t                               service_command_char_handles;           /**< Handles related to the Service Command Characteristic */
    ble_gatts_char_handles_t                               error_char_handles;                     /**< Handles related to the Error Characteristic */
    uint8_t                                                uuid_type;                              /**< BlueTrack Service UUID type. */
    ble_bluetrack_address_write_handler_t                  address_write_handler;                  /**< If not NULL, handler to be called when Address Characteristic is written to. */
    ble_bluetrack_dcc_command_write_handler_t              dcc_command_write_handler;              /**< If not NULL, handler to be called when DCC Command Characteristic is written to. */
    ble_bluetrack_programming_track_select_write_handler_t programming_track_select_write_handler; /**< If not NULL, handler to be called when Programming Track Select Characteristic is written to. */
    ble_bluetrack_stop_write_handler_t                     stop_write_handler;                     /**< If not NULL, handler to be called when Stop Characteristic is written to. */
    ble_bluetrack_service_command_write_handler_t          service_command_write_handler;          /**< If not NULL, handler to be called when Service Command Characteristic is written to. */
} ble_bluetrack_t;


/**@brief Function for initializing the BlueTrack Service.
 *
 * @param[out]  p_bluetrack       BlueTrack structure. This structure will have to be supplied by
 *                                the application. It will be initialized by this function, and will later
 *                                be used to identify this particular service instance.
 * @param[in]   p_bluetrack_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bluetrack_init(ble_bluetrack_t * p_bluetrack, const ble_bluetrack_init_t * p_bluetrack_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the BlueTrack Service.
 *
 * @param[in]   p_ble_evt    Event received from the BLE stack.
 * @param[in]   p_context    BlueTrack Service structure.
 */
void ble_bluetrack_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for updating the Response characteristic.
 *
 * @details The application calls this function after a service command is processed. The Response characteristic is sent to the client.
 *
 * @param[in]   conn_handle     Handle of the peripheral connection to which the notification will be sent.
 * @param[in]   p_bluetrack     BlueTrack Service structure.
 * @param[in]   response_valid  Flag indicating whether the response is valid, i.e. did the DCC decoder respond as expected?
 * @param[in]   response_value  Value of response (to be interpreted contextually by the client).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bluetrack_response_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack, uint8_t response_valid, uint8_t response_value);


/**@brief Function for updating the Programming Track Select characteristic.
 *
 * @details The application calls this function after the Programming Track Select characteristic is written to. The client is notified of the change.
 *
 * @param[in]   conn_handle     Handle of the peripheral connection to which the notification will be sent.
 * @param[in]   p_bluetrack     BlueTrack Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bluetrack_programming_track_select_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack);


/**@brief Function for updating the Stop characteristic.
 *
 * @details The application calls this function after the Stop characteristic is written to. The client is notified of the change.
 *
 * @param[in]   conn_handle     Handle of the peripheral connection to which the notification will be sent.
 * @param[in]   p_bluetrack     BlueTrack Service structure.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bluetrack_stop_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack);


/**@brief Function for setting and updating the Error characteristic.
 *
 * @details The application calls this function to set and update the Error characteristic. The client is notified of the change.
 *
 * @param[in]   conn_handle     Handle of the peripheral connection to which the notification will be sent.
 * @param[in]   p_bluetrack     BlueTrack Service structure.
 * @param[in]   error_code      Error code the application wishes to communicate to the client.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bluetrack_error_update(uint16_t conn_handle, ble_bluetrack_t * p_bluetrack, uint8_t error_code);

#endif // BLE_BLUETRACK_H__

/** @} */
