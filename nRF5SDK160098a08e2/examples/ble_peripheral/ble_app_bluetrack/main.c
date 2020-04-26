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
 * @defgroup ble_app_bluetrack_main main.c
 * @{
 * @ingroup ble_app_bluetrack
 * @brief BlueTrack Hub project main file.
 *
 * This file contains the implementation for the BlueTrack Hub. This implementation
 * receives configuration, relay, and DCC commands through the BlueTrack BLE service,
 * and forwards these commands to the appropriate outputs.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
//#include "nrf_gpio.h"
//#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_scheduler.h"
//#include "softdevice_handler.h"
#include "app_timer.h"
//#include "ble_error_log.h"
//#include "ble_debug_assert_handler.h"
//#include "pstorage.h"
#include "ble_bluetrack.h"
#include "ble_dis.h"
//#include "app_util_platform.h"
//#include "app_uart.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gatts_cache_manager.h"

#define DREKKER_DEVELOPMENT_COMPANY_IDENTIFIER 0x0343                               /**< Assigned by Bluetooth SIG. */            

#define MAIN_DCC_PIN_NO                 2
#define MAIN_I_SENSE_PIN_NO             3
#define BRAKE_N_PIN_NO                  4
#define RELAY_1_PIN_NO                  9
#define RELAY_2_PIN_NO                  10
#define RELAY_3_PIN_NO                  11
#define RELAY_4_PIN_NO                  12
#define RELAY_5_PIN_NO                  13
#define RELAY_6_PIN_NO                  14
#define RELAY_7_PIN_NO                  15
#define RELAY_8_PIN_NO                  16
#define RELAY_9_PIN_NO                  17
#define RELAY_10_PIN_NO                 18
#define RELAY_11_PIN_NO                 19
#define RELAY_12_PIN_NO                 20
#define RELAY_13_PIN_NO                 22
#define RELAY_14_PIN_NO                 23
#define SYNC_PIN_NO                     24
#define STOP_LED_PIN_NO                 25
#define PROG_I_SENSE_PIN                26
#define PROG_DCC_PIN_NO                 27                            
#define THERMAL_N_PIN_NO                28
#define OVER_LED_PIN_NO                 29
#define BLE_LED_PIN_NO                  30
#define PROG_LED_PIN_NO                 31

#define N_OUTPUTS                       14                                          /**< 14 Relay Outputs in total */

#define DCC_ONE_TIME_US                 58                                          /**< 58us (see S-9.1) */
#define DCC_ZERO_TIME_US                100                                         /**< 100us (see S-9.1) */

#define BLE_LED_TOGGLE_TIME_MS          500                                         /**< Advertising LED toggles every 500ms */
//#define ADC_DELTA                       20                                          /**< ADC value above the baseline required to trigger an acknowledge */
                                                                                    /* 10 bit ADC, 1.2V bandgap reference, and 1.74kohm sense resistor gives the following resolutions:
                                                                                     300uA/A (min) sensitivity - 1.497mA
                                                                                     377uA/A (nom) sensitivity - 1.786mA
                                                                                     450uA/A (max) sensitivity - 2.245mA
                                                                                     
                                                                                     20 counts above baseline gives the following deltas:
                                                                                     300uA/A (min) sensitivity - 29.933mA
                                                                                     377uA/A (nom) sensitivity - 35.729mA
                                                                                     450uA/A (max) sensitivity - 44.899mA
                                                                                     
                                                                                     This allows detection of the minimum 60mA delta specified in S-9.2.3 */

//#define MIN_DCC_LEN                     2                                           /**< Minimum length (in bytes) of a DCC command (excluding error byte, see S-9.2.1) */
//#define MAX_DCC_LEN                     5                                           /**< Maximum length (in bytes) of a DCC command (excluding error byte, see S-9.2.1) */

//#define TIMER1_IRQ_PRI                  APP_IRQ_PRIORITY_HIGH                       /**< Set priority of DCC timer to application high */
//#define TIMER1_PRESCALER_VAL            0                                           /**< Set base period to 62.5ns */
//#define DCC_ONE_CC_VAL                  928                                         /**< 928 * 62.5ns = 58us (see S-9.1) */
//#define DCC_ZERO_CC_VAL                 1600                                        /**< 1600 * 62.5ns = 100us (see S-9.1) */

//#define TIMER2_PRESCALER_VAL_LED        7                                           /**< Set base period to 80us when driving an LED */
//#define TIMER2_PRESCALER_VAL_DCC        TIMER1_PRESCALER_VAL                        /**< Set base period to 62.5ns when driving the programming DCC output */
//#define TIMER2_CC_VAL_LED               62500                                       /**< 62500 * 80us = 500ms (advertising LED toggles every 500ms) */

//#define LPCOMP_IRQ_PRI                  APP_IRQ_PRIORITY_HIGH                       /**< Set priority of comparator to application high */

//#define GPIOTE_IRQ_PRI                  APP_IRQ_PRIORITY_HIGH                       /**< Set priority of GPIOTE to application high */

//#define ADC_IRQ_PRI                     APP_IRQ_PRIORITY_HIGH                       /**< Set priority of ADC to application high */

//#define UART_IRQ_PRI                    APP_IRQ_PRIORITY_LOW                        /**< Set priority of UART to application low */

//#define UART_FIFO_TX_SIZE               128                                         /**< UART TX FIFO size (must be power of 2) */
//#define UART_FIFO_RX_SIZE               128                                         /**< UART RX FIFO size (must be power of 2) */

//#define DCC_COMMAND_BUFFER_SIZE         52                                          /**< Size of DCC Command buffer, this is 32+20 for worst case service mode*/

//#define ACTUATION_INTERVAL              1000                                        /**< Relay actuation interval (ms) (1s seems reasonable to guarantee actuation) */

//#define ADC_SAMPLE_INTERVAL             0.5                                         /**< ADC sample interval (ms) (this allows at least 10 samples during a minimum 5ms feedback window, see S-9.2.3) */

//#define FUNCTION_READ_BIT               1                                           /**< Enum for read_bit function */
//#define FUNCTION_READ_BYTE              2                                           /**< Enum for read_byte function */
//#define FUNCTION_WRITE_BIT              3                                           /**< Enum for write_bit function */
//#define FUNCTION_WRITE_BYTE             4                                           /**< Enum for write_byte function */
//#define MODE_DIRECT                     1                                           /**< Enum for direct mode */
//#define MODE_PAGED                      2                                           /**< Enum for paged mode */
//#define MODE_REGISTER                   3                                           /**< Enum for register mode */
//#define MODE_ADDRESS                    4                                           /**< Enum for address mode */
//#define READ_BYTE_COUNTER_MAX           255                                         /**< Maximum value for the read byte counter (maximum value of 8 bit unsigned integer) */
//#define READ_BIT_COUNTER_MAX            7                                           /**< Maximum value for the read bit counter (8-1) */
//#define READ_BIT_COUNTER_MAX_ADDRESS    127                                         /**< Maximum value for the read byte counter when in address mode (short address cannot be greater than 127) */

//#define ADDRESS_TYPE_INVALID            0                                           /**< Enum for invalid address type */
//#define ADDRESS_TYPE_BROADCAST          1                                           /**< Enum for broadcast address type */
//#define ADDRESS_TYPE_SHORT_CONSIST      2                                           /**< Enum for short or consist address type */
//#define ADDRESS_TYPE_LONG               3                                           /**< Enum for long address type */
//
//#define COMMAND_REPEATS                 10                                          /**< Number of times to repeat a DCC command (arbitrary selection) */
//
//#define SPEED_COMMAND_ARRAY_SIZE        128                                         /**< Number of addresses we can store a speed command for. This places a limitation on the number of trains we can concurrently support. */
//
//#define MAIN                            0                                           /**< Indicates DCC commands are sent to the main track. */
//#define PROGRAMMING                     1                                           /**< Indicates DCC commands are sent to the programming track. */
//#define MAIN_REPEAT                     2                                           /**< Indicates repeating DCC commands are being sent to the main track in programming mode */

#define DEVICE_NAME                     "BlueTrack"                                /**< Name of device. Will be included in the advertising data. Limit length to 11 to allow Master Emulator to connect (unexplained). */

#define MODEL_NUMBER                    "BlueTrack Hub"                            /**< Model number for DIS. **/
#define MANUFACTURER_NAME               "Drekker Development Pty. Ltd."            /**< Manufacturer name for DIS. **/

#define APP_BLE_OBSERVER_PRIO           3                                          /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(20, UNIT_0_625_MS )          /**< The advertising interval, 20ms recommended by R12 of Accessory Design Guidelines for Apple Devices */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED      /**< Advertising duration in 10 ms units. */

//#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register (no prescaling, maximum resolution). */
//#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers (only output timer and feedback timer). */
//#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues (only output timer and feedback timer, + 1 for how the queue is implemented). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(510, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.51 seconds to comply with R12 of Accessory Design Guidelines for Apple Devices). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency (0 to comply with R12 of Accessory Design Guidelines for Apple Devices). */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds to comply with R12 of Accessory Design Guidelines for Apple Devices). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

//#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
//#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
//#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
//#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Timer is our largest scheduled event (8 bytes). Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. This is a best guess. */

/**@brief Structure of a DCC command. */
//typedef struct
//{
//    bool occupied;
//    bool sync;
//    bool feedback;
//    uint32_t data0;
//    uint32_t data1;
//    uint32_t data2;
//    uint8_t data0_count;
//    uint8_t data1_count;
//    uint8_t data2_count;
//} dcc_command_t;

/**@brief Structure of a scheduled DCC command. */
//typedef struct
//{
//    uint8_t byte1;
//    uint8_t byte2;
//    uint8_t byte3;
//    uint8_t byte4;
//    uint8_t byte5;
//    uint8_t len;
//    uint8_t feedback;
//} scheduled_dcc_command_t;

BLE_BLUETRACK_DEF(m_bluetrack);                                                 /**< Bluetrack Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/


//static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

const nrf_drv_timer_t TIMER_DCC_DATA =           NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t TIMER_BLE_LED =            NRF_DRV_TIMER_INSTANCE(2);
const nrf_drv_timer_t TIMER_DCC_CONTINOUS_ONES = NRF_DRV_TIMER_INSTANCE(3);

static nrf_ppi_channel_t ble_led_timer_to_ble_led;
static nrf_ppi_channel_t dcc_data_to_main_dcc;
static nrf_ppi_channel_t dcc_data_to_prog_dcc;
static nrf_ppi_channel_t dcc_continuous_ones_to_main_dcc;
static nrf_ppi_channel_t dcc_continuous_ones_to_prog_dcc;

// Persistent storage system event handler
//void pstorage_sys_event_handler (uint32_t p_evt);

//static ble_bluetrack_t                  m_bluetrack;                                          /**< BlueTrack service structure instance */
//static app_timer_id_t                   output_timer;                                         /**< App Timer instance for relay output timing */
//static app_timer_id_t                   feedback_timer;                                       /**< App Timer instance for feedback ADC measurement timing */
static nrf_drv_gpiote_pin_t             output_index[N_OUTPUTS] = {RELAY_1_PIN_NO,            /**< Pin Number storage for the relay outputs */
                                                                   RELAY_2_PIN_NO,
                                                                   RELAY_3_PIN_NO,
                                                                   RELAY_4_PIN_NO,
                                                                   RELAY_5_PIN_NO,
                                                                   RELAY_6_PIN_NO,
                                                                   RELAY_7_PIN_NO,
                                                                   RELAY_8_PIN_NO,
                                                                   RELAY_9_PIN_NO,
                                                                   RELAY_10_PIN_NO,
                                                                   RELAY_11_PIN_NO,
                                                                   RELAY_12_PIN_NO,
                                                                   RELAY_13_PIN_NO,
                                                                   RELAY_14_PIN_NO};
//static bool                             output_flags[N_OUTPUTS];                              /**< Flags for requesting relay activation */
//static dcc_command_t                    dcc_command_buffer[DCC_COMMAND_BUFFER_SIZE];          /**< DCC command buffer that stores pending commands */
//static uint8_t                          producer_index;                                       /**< Index of the next available DCC command buffer slot that can be filled */
//static uint8_t                          consumer_index;                                       /**< Index of the DCC command buffer slot currently being transmitted */
//static bool                             phase1_complete;                                      /**< Flag to control phases of DCC command */
//static bool                             phase2_complete;                                      /**< Flag to control phases of DCC command */
//static bool                             dcc_output_state;                                     /**< Flag to track whether we are currently outputting a DCC 1 or 0 */
//static bool                             dcc_disabled;                                         /**< Flag to indicate whether DCC has been disabled completely */
//static bool                             adc_baseline_flag;                                    /**< Flag to indicate to the ADC it should store its result as a baseline */
//static uint32_t                         adc_baseline;                                         /**< Baseline of current feedback measurement */
//static bool                             feedback_window_end;                                  /**< Flag to indicate that the feedback window should end */
//static bool                             feedback_in_progress;                                 /**< Flag to indicate whether we are monitoring for feedback */
//static bool                             acknowledge;                                          /**< Flag to indicate an acknowledge was received during feedback */
//static bool                             service_command_pending;                              /**< Flag to indicate whether a service command is pending */
//static bool                             service_command_in_progress;                          /**< Flag to indicate whether a service command is in progress */
//static uint8_t                          programming_track_mode;                               /**< Variable to keep track of programming track mode */
//static uint8_t                          programming_track_state;                              /**< Variable to keep track of programming track state */
//static uint8_t                          function;                                             /**< Function of the service command pending/in progress */
//static uint8_t                          mode;                                                 /**< Mode of the service command pending/in progress */
//static uint8_t                          CV_or_reg_upper;                                      /**< Upper portion of CV or register of the service command pending/in progress */
//static uint8_t                          CV_or_reg_lower;                                      /**< Lower portion of CV or register of the service command pending/in progress */
//static uint8_t                          value;                                                /**< Value to be written of the service command pending/in progress */
//static uint16_t                         read_byte_counter;                                    /**< Read byte counter of the service command pending/in progress */
//static uint8_t                          read_bit_counter;                                     /**< Read bit counter of the service command pending/in progress */
//static uint8_t                          read_bit_value;                                       /**< Read bit response of the service command pending/in progress */
//static dcc_command_t                    idle_packet;                                          /**< Memory reserved to transmit idle packets */
//static uint32_t                         active_speed_command_index;                           /**< Current place in the speed command array */
//static dcc_command_t                    speed_command_array_temp[SPEED_COMMAND_ARRAY_SIZE];   /**< Array for sending periodic speed commands */
//static dcc_command_t                    speed_command_array[SPEED_COMMAND_ARRAY_SIZE];        /**< Array for storing periodic speed commands */
//static uint8_t                          speed_command_address_type[SPEED_COMMAND_ARRAY_SIZE]; /**< Array for keeping track of the address type the corresponding periodic speed command applies to */
//static uint16_t                         speed_command_address[SPEED_COMMAND_ARRAY_SIZE];      /**< Array for keeping track of the address the corresponding periodic speed command applies to */


/**@brief Function for removing all periodic speed commands
 */
//static void remove_all_speed_commands (void)
//{
//    uint32_t i;
//    for (i = 0; i < SPEED_COMMAND_ARRAY_SIZE; i++)
//    {
//        speed_command_array[i].occupied = false;
//        speed_command_array_temp[i].occupied = false;
//        speed_command_address_type[i] = ADDRESS_TYPE_INVALID;
//    }
//}


/**@brief Function for disabling the DCC output.
 *
 * @details This function is called whenever the DCC output needs to be halted immediately (for example, in error condition). 
 * DCC output disabling is only resettable by complete system reset. 
 */
//static void disable_DCC (uint8_t error_code)
//{
//    uint32_t err_code;
//    
//    // Raise flag
//    dcc_disabled = true;
//
//    // Turn on brake
//    nrf_gpio_pin_clear(BRAKE_PIN_NO);
//
//    // Indicate on LED
//    nrf_gpio_pin_set(OVERLOAD_LED_PIN_NO);
//    
//    // Notify the client
//    err_code = ble_bluetrack_error_update(&m_bluetrack, error_code);
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for identifying an appropriate index for a periodic speed command. Returns SPEED_COMMAND_ARRAY_SIZE if full.
 */
//static uint8_t index_for_periodic_speed_command(uint8_t address_type, uint16_t address)
//{
//    uint8_t i;
//    uint8_t vacant = SPEED_COMMAND_ARRAY_SIZE;
//    for (i = 0; i < SPEED_COMMAND_ARRAY_SIZE; i++)
//    {
//        if (speed_command_address_type[i] == ADDRESS_TYPE_INVALID)
//        {
//            // We've found a vacant index, store it
//            vacant = i;
//        }
//        else if ((speed_command_address_type[i] == address_type) && (speed_command_address[i] == address))
//        {
//            // We've found a corresponding entry
//            return i;
//        }
//    }
//    
//    // We made it here without finding a corresponding entry, simply return vacant
//    return vacant;
//}


/**@brief Function for handling a write to the Address characteristic.
 *
 * @details This function enqueues the written address for relay actuation, providing it is in range.
 *
 * @param[in]   p_bluetrack    BlueTrack service structure.
 * @param[in]   address        Address of relay output to be actuated.
 */
static void address_write_handler(ble_bluetrack_t * p_bluetrack, uint8_t address)
{
    NRF_LOG_INFO("Address Written");

//    UNUSED_VARIABLE(p_bluetrack);
//
//    // First check address is in range.
//    if (address < N_OUTPUTS)
//    {
//        // Raise flag for actuation
//        output_flags[address] = true;
//    }
}


/**@brief Function for enqueing a DCC command.
 *
 * @details This function places the DCC Command on the buffer, encoding it appropriately. It will silently drop the command if there is insufficient space in the buffer. As a convenience, it will also return the enqueued command by reference (with the occupied flag UNSET).
 *
 * @param[in]   byte1          byte1 of DCC command.
 * @param[in]   byte2          byte2 of DCC command.
 * @param[in]   byte3          byte3 of DCC command.
 * @param[in]   byte4          byte4 of DCC command.
 * @param[in]   byte5          byte5 of DCC command.
 * @param[in]   byte6          byte6 of DCC command.
 * @param[in]   len            Length of DCC command in bytes.
 * @param[in]   feedback       Flag to indicate whether a feedback window should be opened for this command.
 */
//static void enqueue_dcc_command (uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t len, uint8_t feedback, dcc_command_t *enqueued_command)
//{
//    uint8_t next_producer_index;
//    
//    // First check for valid length
//    if (len >= MIN_DCC_LEN && len <= MAX_DCC_LEN)
//    {
//        next_producer_index = producer_index + 1;
//        // Reset next producer index if we have overflowed
//        if (next_producer_index == DCC_COMMAND_BUFFER_SIZE)
//        {
//            next_producer_index = 0;
//        }
//        
//        // Only transmit if we have enough room to move to the next slot
//        if (next_producer_index != consumer_index)
//        {
//            // Send a DCC packet (long preamble (20 bits) used to satisfy S-9.2.3)
//            if (len == 2)
//            {
//                dcc_command_buffer[producer_index].data0 = 0xFFFFF000 | (byte1 << 3);
//                dcc_command_buffer[producer_index].data0_count = 30;
//                dcc_command_buffer[producer_index].data1 = 0x00004000 | (byte2 << 24) | ((byte1 ^ byte2) << 15);
//                dcc_command_buffer[producer_index].data1_count = 18;
//                dcc_command_buffer[producer_index].data2 = 0x80000000;
//                dcc_command_buffer[producer_index].data2_count = 1;
//            }
//            else if (len == 3)
//            {
//                dcc_command_buffer[producer_index].data0 = 0xFFFFF000 | (byte1 << 3);
//                dcc_command_buffer[producer_index].data0_count = 30;
//                dcc_command_buffer[producer_index].data1 = 0x00000020 | (byte2 << 24) | (byte3 << 15) | ((byte1 ^ byte2 ^ byte3) << 6);
//                dcc_command_buffer[producer_index].data1_count = 27;
//                dcc_command_buffer[producer_index].data2 = 0x80000000;
//                dcc_command_buffer[producer_index].data2_count = 1;
//            }
//            else if (len == 4)
//            {
//                dcc_command_buffer[producer_index].data0 = 0xFFFFF000 | (byte1 << 3);
//                dcc_command_buffer[producer_index].data0_count = 30;
//                dcc_command_buffer[producer_index].data1 = 0x00000000 | (byte2 << 24) | (byte3 << 15) | ((byte4) << 6);
//                dcc_command_buffer[producer_index].data1_count = 27;
//                dcc_command_buffer[producer_index].data2 = 0x00800000 | ((byte1 ^ byte2 ^ byte3 ^ byte4) << 24);
//                dcc_command_buffer[producer_index].data2_count = 9;
//            }
//            else if (len == 5)
//            {
//                dcc_command_buffer[producer_index].data0 = 0xFFFFF000 | (byte1 << 3);
//                dcc_command_buffer[producer_index].data0_count = 30;
//                dcc_command_buffer[producer_index].data1 = 0x00000000 | (byte2 << 24) | (byte3 << 15) | ((byte4) << 6);
//                dcc_command_buffer[producer_index].data1_count = 27;
//                dcc_command_buffer[producer_index].data2 = 0x00004000 | (byte5 << 24) | ((byte1 ^ byte2 ^ byte3 ^ byte4 ^ byte5) << 15);
//                dcc_command_buffer[producer_index].data2_count = 18;
//            }
//            
//            dcc_command_buffer[producer_index].sync = true;
//            dcc_command_buffer[producer_index].feedback = feedback ? true : false;
//            
//            // Return by reference, if required
//            if (enqueued_command != NULL)
//            {
//                memcpy(enqueued_command, &dcc_command_buffer[producer_index], sizeof(dcc_command_t));
//            }
//            
//            dcc_command_buffer[producer_index].occupied = true;
//            
//            // Advance producer index
//            producer_index = next_producer_index;
//        }
//    }
//}


/**@brief Function for enqueing a DCC command from the scheduler.
 *
 * @details This function calls enqueue_dcc_command.
 *
 * @param[in]   p_event_data   Pointer to scheduled_dcc_command data type.
 * @param[in]   event_size     Size of scheduled_dcc_command.
 */
//void enqueue_dcc_command_from_scheduler (void *p_event_data, uint16_t event_size)
//{
//    uint8_t i;
//    uint16_t address = 0;
//    uint8_t address_type = ADDRESS_TYPE_INVALID;
//    uint8_t command_byte, data_byte;
//    uint8_t index;
//    dcc_command_t speed_command;
//    scheduled_dcc_command_t *p_scheduled_dcc_command;
//    
//    if (event_size == sizeof(scheduled_dcc_command_t))
//    {
//        p_scheduled_dcc_command = (scheduled_dcc_command_t *)p_event_data;
//        
//        // We send the command first to get a copy of the enqueued packet for the periodic speed command. We think we can do this because we assume this function will purge the periodic speed command array before the sent command (and its repetitions) makes it out on the rails, so if a stop is sent, the previous periodic speed command for that address will be purged before the DCC timer handler gets around to pulling it from the periodic speed command array after it clears the DCC buffer.
//        
//        // Repeat the command as required
//        for (i = 0; i < COMMAND_REPEATS; i++)
//        {
//            enqueue_dcc_command(p_scheduled_dcc_command->byte1, p_scheduled_dcc_command->byte2, p_scheduled_dcc_command->byte3, p_scheduled_dcc_command->byte4, p_scheduled_dcc_command->byte5, p_scheduled_dcc_command->len, p_scheduled_dcc_command->feedback, &speed_command);
//        }
//        
//        // If we have a reset or hard reset packet in operations mode sent to broadcast, short, or long addresses, queue up 10 idle packets to ensure the decoders don't enter service mode unintentionally
//        if (((p_scheduled_dcc_command->byte1 <= 127) && ((p_scheduled_dcc_command->byte2 == 0x00) || (p_scheduled_dcc_command->byte2 == 0x01))) ||
//            ((p_scheduled_dcc_command->byte1 >= 192) && (p_scheduled_dcc_command->byte1 <= 231) && ((p_scheduled_dcc_command->byte3 == 0x00) || (p_scheduled_dcc_command->byte3 == 0x01))))
//        {
//            for (i = 0; i < 10; i++)
//            {
//                enqueue_dcc_command(0xFF, 0, 0, 0, 0, 2, 0, NULL);
//            }
//        }
//        
//        // Only update the periodic speed commands when we are in main track mode
//        if (programming_track_mode == MAIN)
//        {
//            // First identify the address (if valid)
//            if (p_scheduled_dcc_command->byte1 == 0)
//            {
//                address_type = ADDRESS_TYPE_BROADCAST;
//                address = p_scheduled_dcc_command->byte1;
//                command_byte = p_scheduled_dcc_command->byte2;
//                data_byte = p_scheduled_dcc_command->byte3;
//            }
//            else if ((p_scheduled_dcc_command->byte1 >= 1) && (p_scheduled_dcc_command->byte1 <= 127))
//            {
//                address_type = ADDRESS_TYPE_SHORT_CONSIST;
//                address = p_scheduled_dcc_command->byte1;
//                command_byte = p_scheduled_dcc_command->byte2;
//                data_byte = p_scheduled_dcc_command->byte3;
//            }
//            else if ((p_scheduled_dcc_command->byte1 >= 192) && (p_scheduled_dcc_command->byte1 <= 231))
//            {
//                address_type = ADDRESS_TYPE_LONG;
//                address = ((p_scheduled_dcc_command->byte1 & 0x3F) << 8) | p_scheduled_dcc_command->byte2;
//                command_byte = p_scheduled_dcc_command->byte3;
//                data_byte = p_scheduled_dcc_command->byte4;
//            }
//            
//            // Now check if we have a speed command
//            if (address_type != ADDRESS_TYPE_INVALID)
//            {
//                if (((command_byte == 0x3F) && ((data_byte & 0x7E) == 0)) || (((command_byte & 0xC0) == 0x40) && ((command_byte & 0x0E) == 0)))
//                {
//                    // We have a 128 step speed command or a 28 step speed command which is a stop or emergency stop
//                    if (address_type == ADDRESS_TYPE_BROADCAST)
//                    {
//                        // For a broadcast command, wipe the periodic speed commands
//                        remove_all_speed_commands();
//                    }
//                    else
//                    {
//                        // We only neeed to wipe the command of the affected address
//                        index = index_for_periodic_speed_command(address_type, address);
//                        if (index != SPEED_COMMAND_ARRAY_SIZE)
//                        {
//                            // We are safe to wipe a vacant location, if that is what was returned
//                            speed_command_array[index].occupied = false;
//                            speed_command_array_temp[index].occupied = false;
//                            speed_command_address_type[index] = ADDRESS_TYPE_INVALID;
//                        }
//                    }
//                }
//                else if (((command_byte == 0x3F) && ((data_byte & 0x7E) != 0)) || (((command_byte & 0xC0) == 0x40) && ((command_byte & 0x0E) != 0)))
//                {
//                    // We have a 128 step speed command or a 28 step speed command which is a speed command
//                    // Note we do this for all address types, including broadcast; the logic here is if someone is silly enough to do a broadcast speed command, it should still be periodicallly repeated
//                    index = index_for_periodic_speed_command(address_type, address);
//                    if (index != SPEED_COMMAND_ARRAY_SIZE)
//                    {
//                        // We are safe to add/overwrite
//                        speed_command_address_type[index] = address_type;
//                        speed_command_address[index] = address;
//                        speed_command_array[index].occupied = false;
//                        speed_command_array_temp[index].occupied = false;
//                        memcpy(&speed_command_array[index], &speed_command, sizeof(dcc_command_t));
//                        memcpy(&speed_command_array_temp[index], &speed_command, sizeof(dcc_command_t));
//                        speed_command_array[index].occupied = true;
//                        speed_command_array_temp[index].occupied = true;
//                    }
//                    else
//                    {
//                        // There was not enough room in the array, raise an error
//                        disable_DCC(ERROR_CODE_MAX_TRAIN_EXCEEDED);
//                    }
//                }
//            }
//        }
//    }
//    else
//    {
//        APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
//    }
//}


/**@brief Function for handling a write to the DCC Command characteristic.
 *
 * @details This function places the DCC Command on the buffer, encoding it appropriately. It will silently drop the command if there is insufficient space in the buffer.
 *
 * @param[in]   p_bluetrack    BlueTrack service structure.
 * @param[in]   byte1          byte1 of DCC command (MSB).
 * @param[in]   byte2          byte2 of DCC command.
 * @param[in]   byte3          byte3 of DCC command.
 * @param[in]   byte4          byte4 of DCC command.
 * @param[in]   byte5          byte5 of DCC command.
 * @param[in]   byte6          byte6 of DCC command.
 * @param[in]   len            Length of DCC command in bytes.
 * @param[in]   feedback       Flag to indicate whether a feedback window should be opened for this command (LSB).
 */
static void dcc_command_write_handler(ble_bluetrack_t * p_bluetrack, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t len)
{
    NRF_LOG_INFO("DCC Command Written");

//    scheduled_dcc_command_t scheduled_dcc_command;
//    uint32_t err_code;
//    UNUSED_VARIABLE(p_bluetrack);
//    
//    // Only schedule the enqueue of the command if we have no service command pending or in progress
//    if (!service_command_pending && !service_command_in_progress)
//    {
//        scheduled_dcc_command.byte1 = byte1;
//        scheduled_dcc_command.byte2 = byte2;
//        scheduled_dcc_command.byte3 = byte3;
//        scheduled_dcc_command.byte4 = byte4;
//        scheduled_dcc_command.byte5 = byte5;
//        scheduled_dcc_command.len = len;
//        scheduled_dcc_command.feedback = 0;
//        
//        err_code = app_sched_event_put(&scheduled_dcc_command, sizeof(scheduled_dcc_command_t), enqueue_dcc_command_from_scheduler);
//        APP_ERROR_CHECK(err_code);
//    }
}


/**@brief Function for notifying the programming track select characteristic.
 *
 * @details This function posts a notification to the programming track select characteristic. It is done outside the write handler as an error was observed previously in this context.
 *
 * @param[in]   p_event_data   Undefined pointer.
 * @param[in]   event_size     Size of 0.
 */
//void programming_track_select_notify (void *p_event_data, uint16_t event_size)
//{
//    uint32_t err_code;
//
//    if (event_size == 0)
//    {
//        err_code = ble_bluetrack_programming_track_select_update(&m_bluetrack);
//        APP_ERROR_CHECK(err_code);
//    }
//    else
//    {
//        APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
//    }
//}


/**@brief Function for notifying the stop characteristic.
 *
 * @details This function posts a notification to the stop characteristic. It is done outside the write handler as an error was observed previously in this context
 *
 * @param[in]   p_event_data   Undefined pointer.
 * @param[in]   event_size     Size of 0.
 */
//void stop_notify (void *p_event_data, uint16_t event_size)
//{
//    uint32_t err_code;
//
//    if (event_size == 0)
//    {
//        err_code = ble_bluetrack_stop_update(&m_bluetrack);
//        APP_ERROR_CHECK(err_code);
//    }
//    else
//    {
//        APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
//    }
//}


/**@brief Function for handling a write to the Programming Track Select characteristic.
 *
 * @details This function enters programming mode if select is true by enabling the programming track output and indicating via LED.
 *          We do not check for 250mA current sustained for 100ms, which is optional (should) in S-9.2.3.
 *
 * @param[in]   p_bluetrack    BlueTrack service structure.
 * @param[in]   select         Flag to indicate whether the programming track should be selected.
 */
static void programming_track_select_write_handler(ble_bluetrack_t * p_bluetrack, uint8_t select)
{
    NRF_LOG_INFO("Programming Track Select Written");

//    UNUSED_VARIABLE(p_bluetrack);
//    uint32_t err_code;
//    
//    if (select == MAIN)
//    {
//        programming_track_mode = MAIN;
//
//        // Indicate programming track deselection
//        nrf_gpio_pin_clear(PROGRAMMING_LED_PIN_NO);
//    }
//    else
//    {
//        programming_track_mode = PROGRAMMING;
//
//        // Indicate programming track selection
//        nrf_gpio_pin_set(PROGRAMMING_LED_PIN_NO);
//    }
//
//    // Notify the client
//    err_code = app_sched_event_put(NULL, 0, programming_track_select_notify);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling a write to the Stop characteristic.
 *
 * @details This function halts DCC output if select is true by disabling the DCC pin, turning on the brake, and indicating via LED. It starts it again by reversing these conditions, 
 * only if the DCC output has not already been disabled.
 *
 * @param[in]   p_bluetrack    BlueTrack service structure.
 * @param[in]   select         Flag to indicate whether DCC output should be stopped.
 */
static void stop_write_handler(ble_bluetrack_t * p_bluetrack, uint8_t stop)
{
    NRF_LOG_INFO("Stop Written");

//    UNUSED_VARIABLE(p_bluetrack);
//    uint32_t err_code;
//
//    // First clear all periodic speed commands
//    remove_all_speed_commands();
//    
//    if (stop)
//    {
//        // Turn on brake
//        nrf_gpio_pin_clear(BRAKE_PIN_NO);
//
//        // Indicate on LED
//        nrf_gpio_pin_set(STOP_LED_PIN_NO);
//    }
//    else if (!stop)
//    {
//        // Indicate on LED
//        nrf_gpio_pin_clear(STOP_LED_PIN_NO);
//
//        // Only start DCC output if disabled flag is not set
//        if (!dcc_disabled)
//        {
//            // Release the brake
//            nrf_gpio_pin_set(BRAKE_PIN_NO);
//        }
//    }
//
//    // Notify the client
//    err_code = app_sched_event_put(NULL, 0, stop_notify);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling a write to the Service Command characteristic.
 *
 * @details This function sets up global variables to precipitate a series of service command sequences.
 *
 * @param[in]   p_bluetrack            BlueTrack service structure.
 * @param[in]   function_local         Indicates whether this is a read_bit, read_byte, write_bit, or write_byte series.
 * @param[in]   mode_local             Indicates whether the access mode is direct, paged, register, or address.
 * @param[in]   CV_or_reg_upper_local  The upper portion of the CV or register address to be accessed.
 * @param[in]   CV_or_reg_lower_local  The lower portion of the CV or register address to be accessed.
 * @param[in]   value_local            The value to be written to the CV or register address if write_byte series.
 */
static void service_command_write_handler(ble_bluetrack_t * p_bluetrack, uint8_t function_local, uint8_t mode_local, uint8_t CV_or_reg_upper_local, uint8_t CV_or_reg_lower_local, uint8_t value_local)
{
    NRF_LOG_INFO("Service Command Written");

//    UNUSED_VARIABLE(p_bluetrack);
//    if (!service_command_pending && !service_command_in_progress)
//    {
//        // Initialise counters and values
//        read_byte_counter = 0;
//        read_bit_counter = 0;
//        read_bit_value = 0;
//        
//        // Store command parameters
//        function = function_local;
//        mode = mode_local;
//        CV_or_reg_upper = CV_or_reg_upper_local;
//        CV_or_reg_lower = CV_or_reg_lower_local;
//        value = value_local;
//        
//        // Raise pending flag only if parameters are valid
//        if ((function == FUNCTION_READ_BIT || function == FUNCTION_READ_BYTE || function == FUNCTION_WRITE_BIT || function == FUNCTION_WRITE_BYTE) &&
//            (mode == MODE_DIRECT || mode == MODE_ADDRESS || mode == MODE_PAGED || mode == MODE_REGISTER) &&
//            ((function != FUNCTION_READ_BIT && function != FUNCTION_WRITE_BIT) || ((function == FUNCTION_READ_BIT || function == FUNCTION_WRITE_BIT) && (mode == MODE_DIRECT))))
//        {
//            service_command_pending = true;
//        }
//    }
}


/**@brief Function for executing a service command.
 *
 * @details This function places a series of DCC commands in the buffer corresponding to the service command requested, as
 * indicated by the relevant global variables. Every service command sequence must have a feedback request.
 *
 * @param[in]   p_event_data   Undefined pointer.
 * @param[in]   event_size     Size of 0.
 */
//void execute_service_command (void *p_event_data, uint16_t event_size)
//{
//    uint8_t byte1;
//    uint8_t byte2;
//    uint8_t byte3;
//
//    uint8_t i;
//
//    uint8_t page;
//    uint8_t data_register;
//    
//    uint16_t CV_or_reg = ( ((uint16_t)CV_or_reg_lower) | 
//                 ( ((uint16_t)CV_or_reg_upper) << 8 ) );
//#ifdef DEBUG
//    uint32_t err_code;
//#endif
//    if (event_size == 0)
//    {
//        // As per S-9.2.3, we should guarantee a power on cycle by queueing 20 idle packets
//        for (i = 0; i < 20; i++)
//        {
//            enqueue_dcc_command(0xFF, 0, 0, 0, 0, 2, 0, NULL);
//        }
//        
//        if (function == FUNCTION_WRITE_BYTE)
//        {
//            if (mode == MODE_DIRECT)
//            {
//                // TOTAL 14
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // We are sending a CV address, so only extract the lower two bits from the upper byte
//                byte1 = 0x7C | (CV_or_reg_upper & 0x03);
//                byte2 = CV_or_reg_lower;
//                byte3 = value;
//                
//                // 5 write packets
//                enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//            else if (mode == MODE_ADDRESS)
//            {
//                // TOTAL 32
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, 0x01, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                byte1 = 0x78;
//                // Ensure the address to be written is <= 127 (S-9.2.2 CV 1)
//                byte2 = value & 0x7F;
//                
//                // 5 write packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 10 reset packets
//                for (i = 0; i < 10; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//            else if (mode == MODE_REGISTER)
//            {
//                // TOTAL 32
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, 0x01, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // For a register address, extract only the lower three bits from the lower byte
//                byte1 = 0x78 | (CV_or_reg_lower & 0x07);
//                byte2 = value;
//                
//                // 5 write packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 10 reset packets
//                for (i = 0; i < 10; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//            else if (mode == MODE_PAGED)
//            {
//                // TOTAL 28
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // Ensure CV is < 1024 before converting to page number
//                page = ((CV_or_reg & 0x03FF) / 4) + 1;
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, page, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // Ensure CV is < 1024 before converting to register number
//                data_register = (CV_or_reg & 0x03FF) % 4;
//                byte1 = 0x78 | data_register;
//                byte2 = value;
//                
//                // 5 write packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//        }
//        else if (function == FUNCTION_READ_BYTE)
//        {
//            if (mode == MODE_DIRECT)
//            {
//                // TOTAL 9
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // We are sending a CV address, so only extract the lower two bits from the upper byte
//                byte1 = 0x74 | (CV_or_reg_upper & 0x03);
//                byte2 = CV_or_reg_lower;
//                byte3 = read_byte_counter;
//                
//                // 5 read packets
//                enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 0, NULL);
//                }
//                
//                // 1 reset packet
//                enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//            }
//            else if (mode == MODE_ADDRESS)
//            {
//                // TOTAL 21
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, 0x01, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 5 reset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                byte1 = 0x70;
//                // Ensure the address to be written is <= 127
//                byte2 = read_byte_counter & 0x7F;
//                
//                // 5 read packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//            else if (mode == MODE_REGISTER)
//            {
//                // TOTAL 24
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, 0x01, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // For a register address, extract only the lower three bits from the lower byte
//                byte1 = 0x70 | (CV_or_reg_lower & 0x07);
//                byte2 = read_byte_counter;
//                
//                // 7 read packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 7; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//            else if (mode == MODE_PAGED)
//            {
//                // TOTAL 22
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // Ensure CV is < 1024 before converting to page number
//                page = ((CV_or_reg & 0x03FF) / 4) + 1;
//                
//                // 5 page preset packets
//                for (i = 0; i < 5; i++)
//                {
//                    enqueue_dcc_command(0x7D, page, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // Ensure CV is < 1024 before converting to register number
//                data_register = (CV_or_reg & 0x03FF) % 4;
//                byte1 = 0x70 | data_register;
//                byte2 = value;
//                
//                // 5 read packets
//                enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//        }
//        else if (function == FUNCTION_WRITE_BIT)
//        {
//            if (mode == MODE_DIRECT)
//            {
//                // TOTAL 14
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // We are sending a CV address, so only extract the lower two bits from the upper byte
//                byte1 = 0x78 | (CV_or_reg_upper & 0x03);
//                byte2 = CV_or_reg_lower;
//                // The lower 4 bits of value are the bit value to be written (bit 3) and the bit position (bits 2-0)
//                byte3 = 0xF0 | (value & 0x0F);
//                
//                // 5 write packets
//                enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 0, NULL);
//                }
//                
//                // 6 reset packets
//                for (i = 0; i < 6; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//            }
//        }
//        else if (function == FUNCTION_READ_BIT)
//        {
//            if (mode == MODE_DIRECT)
//            {
//                // TOTAL 9
//                // 3 reset packets
//                for (i = 0; i < 3; i++)
//                {
//                    enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//                }
//                
//                // We are sending a CV address, so only extract the lower two bits from the upper byte
//                byte1 = 0x78 | (CV_or_reg_upper & 0x03);
//                byte2 = CV_or_reg_lower;
//                // read_bit_counter is the bit position
//                byte3 = 0xE8 | (read_bit_counter & 0x07);
//                
//                // 5 read packets
//                enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 1, NULL);
//                for (i = 0; i < 4; i++)
//                {
//                    enqueue_dcc_command(byte1, byte2, byte3, 0, 0, 3, 0, NULL);
//                }
//                
//                // 1 reset packet
//                enqueue_dcc_command(0, 0, 0, 0, 0, 2, 0, NULL);
//            }
//        }
//#ifdef DEBUG
//        err_code = app_uart_put_string("Service command enqueued\n\r");
//        APP_ERROR_CHECK(err_code);
//#endif
//    }
//    else
//    {
//        APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
//    }
//}


/**@brief Function for handling the THERMAL_N signal going high to low.
 *
 * @details This indicates overtemperature, therefore disable DCC.
 */
void thermal_n_hi_to_lo_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    NRF_LOG_INFO("THERMAL_N signal went high to low");
    // Disable DCC
    //disable_DCC(ERROR_CODE_OVERTEMPERATURE);
}


/**@brief Function for handling the LPCOMP interrupt.
 *
 * @details The LPCOMP interrupt handler will only be called on a READY or UP event from the comparator. In both these cases, check the result of the comparison.
 * For the READY case, this checks if the comparator is already HIGH on activation. For the UP case, this provides a "debounce" of sorts. If the result is HIGH, 
 * disable DCC, as we are in an overcurrent condition. 
 */
//void LPCOMP_IRQHandler(void)
//{
//    // Clear events
//    NRF_LPCOMP->EVENTS_READY = 0;
//    NRF_LPCOMP->EVENTS_UP = 0;
//
//    // We are either ready, or we observed a up crossing; obtain sample
//    NRF_LPCOMP->TASKS_SAMPLE = 1;
//    if (NRF_LPCOMP->RESULT)
//    {
//        // Disable DCC
//        disable_DCC(ERROR_CODE_OVERCURRENT);
//    }
//}


/**@brief Function for handling the ADC interrupt.
 *
 * @details The ADC interrupt handler will only be called on an END event. In this case, the result is read. Depending on the state of the application flags, 
 * this result is either stored as the baseline of a feedback collection, or compared to the baseline to determine whether the acknowledge flag should be raised.
 */
//void ADC_IRQHandler(void)
//{
//    // Clear event
//    NRF_ADC->EVENTS_END = 0;
//
//    // Record or compare to baseline
//    if (adc_baseline_flag)
//    {
//        adc_baseline = NRF_ADC->RESULT;
//        adc_baseline_flag = false;
//    }
//    else
//    {
//        // Set acknowledge if the result was the required amount above the baseline
//        if (NRF_ADC->RESULT > (adc_baseline + ADC_DELTA))
//        {
//            acknowledge = true;
//        }
//    }
//}


/**@brief Dummy function for handling timer events.
 *
 * @details This handler does nothing.
 */
void dummy_timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
      NRF_LOG_INFO("Dummy timer event triggered");
}


/**@brief Function for handling the DCC_DATA timer event.
 *
 * @details This handler drives the DCC output logic. Based on the phase flags, and the state of the buffer, appropriate values are loaded into TIMER1 CC[0]
 * register correponding to durations of DCC 1s and 0s. A sync output is triggered at the start of each packet. If required, feedback is collected.
 */
void timer_dcc_data_event_handler(nrf_timer_event_t event_type, void* p_context)
{
//    uint32_t err_code;
//    dcc_command_t *active_packet = NULL;
//    bool dcc_packet = false;
//    uint32_t i;
//
//    // Clear events
//    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
//
//    if (phase1_complete)
//    {
//        // We have ended phase 1, timer value remains the same for phase 2
//        phase1_complete = false;
//        phase2_complete = true;
//    }
//    else if (phase2_complete)
//    {
//        // We have ended phase 2, timer value update may be required
//        // First check if there is a pending packet (idle, repeat command, or from the buffer); a packet from the buffer takes priority
//        if (dcc_command_buffer[consumer_index].occupied && !((programming_track_mode == PROGRAMMING) && (programming_track_state == MAIN_REPEAT)))
//        {
//            active_packet = &dcc_command_buffer[consumer_index];
//            dcc_packet = true;
//        }
//        else if (speed_command_array_temp[active_speed_command_index].occupied && !((programming_track_mode == PROGRAMMING) && (programming_track_state == PROGRAMMING)))
//        {
//            active_packet = &speed_command_array_temp[active_speed_command_index];
//        }
//        else if (idle_packet.occupied && !((programming_track_mode == PROGRAMMING) && (programming_track_state == MAIN_REPEAT)))
//        {
//            active_packet = &idle_packet;
//        }
//        
//        if (active_packet)
//        {
//            // Clear the sync output
//            nrf_gpio_pin_clear(SYNC_PIN_NO);
//            // Enable sync output if this is the first bit of this packet
//            if (active_packet->sync)
//            {
//                nrf_gpio_pin_set(SYNC_PIN_NO);
//                active_packet->sync = false;
//            }
//
//            // Only start feedback if feedback is not in progress
//            if (active_packet->feedback && !feedback_in_progress)
//            {
//                // Set variables
//                feedback_in_progress = true;
//                adc_baseline_flag = true;
//                feedback_window_end = false;
//                acknowledge = false;
//                // Start ADC
//                NRF_ADC->TASKS_START = 1;
//                // Start timer
//                err_code = app_timer_start(feedback_timer, APP_TIMER_TICKS(ADC_SAMPLE_INTERVAL, APP_TIMER_PRESCALER), NULL);
//                APP_ERROR_CHECK(err_code);
//            }
//            // Clear feedback flag, we do not want to check it again for this packet
//            active_packet->feedback = false;
//
//            // Determine data for transmission
//            if (active_packet->data0_count > 0)
//            {
//                dcc_output_state = BIT_31 & active_packet->data0;
//                active_packet->data0 = (active_packet->data0 << 1);
//                active_packet->data0_count = active_packet->data0_count - 1;
//            }
//            else if (active_packet->data1_count > 0)
//            {
//                dcc_output_state = BIT_31 & active_packet->data1;
//                active_packet->data1 = (active_packet->data1 << 1);
//                active_packet->data1_count = active_packet->data1_count - 1;
//            }
//            else if (active_packet->data2_count > 0)
//            {
//                dcc_output_state = BIT_31 & active_packet->data2;
//                active_packet->data2 = (active_packet->data2 << 1);
//                active_packet->data2_count = active_packet->data2_count - 1;
//            }
//            else
//            {
//                APP_ERROR_CHECK(NRF_ERROR_INVALID_FLAGS);
//            }
//            // Send data
//            phase1_complete = true;
//            phase2_complete = false;
//            NRF_TIMER1->CC[0] = dcc_output_state ? DCC_ONE_CC_VAL : DCC_ZERO_CC_VAL;
//
//            // Check for exhaustion and unset occupied
//            if (active_packet->data2_count == 0)
//            {
//                active_packet->occupied = false;
//                
//                if (dcc_packet)
//                {
//                    // If it was a DCC packet, update global variables
//                    consumer_index = consumer_index + 1;
//                    // Reset consumer index if we have overflowed
//                    if (consumer_index == DCC_COMMAND_BUFFER_SIZE)
//                    {
//                        consumer_index = 0;
//                    }
//                    
//                    // If we were in a service command and the buffer is empty, indicate to the feedback timer that the buffer is empty and the service command is complete.
//                    // This ensures we continue to look for basic acknowledgement pulses through the decoder recovery time (S-9.2.3)
//                    // NOTE: This ASSUMES all service commands include a feedback command at some point, as the feedback timer is the only thing that can turn service_command_in_progress off
//                    // ASSUME idle packets and speed command packets never request feedback
//                    if (!dcc_command_buffer[consumer_index].occupied && service_command_in_progress)
//                    {
//                        feedback_window_end = true;
//                    }
//                }
//            }
//        }
//        else
//        {
//            // There is nothing to send in any of the packet sources
//            // We're guaranteed to have an empty buffer, so execute a service command, if one is pending
//            if (service_command_pending) {
//                service_command_in_progress = true;
//                service_command_pending = false;
//                err_code = app_sched_event_put(NULL, 0, execute_service_command);
//                APP_ERROR_CHECK(err_code);
//#ifdef DEBUG
//                err_code = app_uart_put_string("Execute service command\n\r");
//                APP_ERROR_CHECK(err_code);
//#endif
//            }
//            
//            // There was no valid speed command packet at the current index (or we were inhibited from sending it), so update the temporary array for that index, increment (to avoid fixating on a single valid entry) and identify the next valid one
//
//            // Copy the speed command back over, making sure to set occupied last (a process removing a speed command will set occupied false first)
//            speed_command_array_temp[active_speed_command_index].data0 = speed_command_array[active_speed_command_index].data0;
//            speed_command_array_temp[active_speed_command_index].data0_count = speed_command_array[active_speed_command_index].data0_count;
//            speed_command_array_temp[active_speed_command_index].data1 = speed_command_array[active_speed_command_index].data1;
//            speed_command_array_temp[active_speed_command_index].data1_count = speed_command_array[active_speed_command_index].data1_count;
//            speed_command_array_temp[active_speed_command_index].data2 = speed_command_array[active_speed_command_index].data2;
//            speed_command_array_temp[active_speed_command_index].data2_count = speed_command_array[active_speed_command_index].data2_count;
//            
//            speed_command_array_temp[active_speed_command_index].sync = speed_command_array[active_speed_command_index].sync;
//            speed_command_array_temp[active_speed_command_index].feedback = speed_command_array[active_speed_command_index].feedback;
//            speed_command_array_temp[active_speed_command_index].occupied = speed_command_array[active_speed_command_index].occupied;
//            
//            // Increment the active speed command index so we don't repeat the command over and over
//            active_speed_command_index = active_speed_command_index + 1;
//            // Reset active speed command index if we have overflowed
//            if (active_speed_command_index == SPEED_COMMAND_ARRAY_SIZE)
//            {
//                active_speed_command_index = 0;
//            }
//            
//            // Now go through the array once to find the next valid packet
//            for (i = 0; i < SPEED_COMMAND_ARRAY_SIZE; i++)
//            {
//                if (speed_command_array_temp[active_speed_command_index].occupied)
//                {
//                    // We've found a valid packet
//                    break;
//                }
//                else
//                {
//                    active_speed_command_index = active_speed_command_index + 1;
//                    // Reset active speed command index if we have overflowed
//                    if (active_speed_command_index == SPEED_COMMAND_ARRAY_SIZE)
//                    {
//                        active_speed_command_index = 0;
//                    }
//                }
//            }
//            
//            // The idle packet needs refreshing (or we were inhibited from sending it)
//            idle_packet.data0 = 0xFFFFF000 | (0xFF << 3);
//            idle_packet.data0_count = 30;
//            idle_packet.data1 = 0x00004000 | (0x00 << 24) | ((0xFF ^ 0x00) << 15);
//            idle_packet.data1_count = 18;
//            idle_packet.data2 = 0x80000000;
//            idle_packet.data2_count = 1;
//            
//            idle_packet.sync = false;
//            idle_packet.feedback = false;
//            idle_packet.occupied = true;
//            
//            // Now update the programming state
//            if ((programming_track_mode == MAIN) && (programming_track_state != MAIN))
//            {
//                // Disable OUT[2] and OUT[3] GPIOTE
//                NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                
//                NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                
//                // Set DCC output to OUT[2] (controlled by TIMER1) and DCC programming output to OUT[3] (continuous 1s)
//                NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                NRF_GPIOTE->CONFIG[2] |= DCC_COMMAND_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                
//                NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                NRF_GPIOTE->CONFIG[3] |= DCC_COMMAND_PROG_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                
//                // Enable OUT[2] and OUT[3] GPIOTE
//                NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//                
//                NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//
//                programming_track_state = MAIN;
//            }
//            else if (programming_track_mode == PROGRAMMING)
//            {
//                if (programming_track_state == PROGRAMMING)
//                {
//                    // Disable OUT[2] and OUT[3] GPIOTE
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    // Set DCC output to OUT[2] (controlled by TIMER1) and DCC programming output to OUT[3] (continuous 1s)
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= DCC_COMMAND_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= DCC_COMMAND_PROG_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                    
//                    // Enable OUT[2] and OUT[3] GPIOTE
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//
//                    programming_track_state = MAIN_REPEAT;
//                }
//                else
//                {
//                    // Disable OUT[2] and OUT[3] GPIOTE
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    // Set DCC programming output to OUT[2] (controlled by TIMER1) and DCC output to OUT[3] (continuous 1s)
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= DCC_COMMAND_PROG_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_PSEL_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= DCC_COMMAND_PIN_NO << GPIOTE_CONFIG_PSEL_Pos;
//                    
//                    // Enable OUT[2] and OUT[3] GPIOTE
//                    NRF_GPIOTE->CONFIG[2] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[2] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//                    
//                    NRF_GPIOTE->CONFIG[3] &= ~GPIOTE_CONFIG_MODE_Msk;
//                    NRF_GPIOTE->CONFIG[3] |= GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos;
//
//                    programming_track_state = PROGRAMMING;
//                }
//            }
//
//            // Send a single one, as we have no data to send this time
//            dcc_output_state = true;
//            phase1_complete = true;
//            phase2_complete = false;
//            NRF_TIMER1->CC[0] = DCC_ONE_CC_VAL;
//        }
//    }
//    else
//    {
//        APP_ERROR_CHECK(NRF_ERROR_INVALID_FLAGS);
//    }
}


/**@brief Function for handling the feedback application timer expiry.
 *
 * @details Feedback application timer runs continuously every feedback window until the DCC buffer is empty. Every time it completes, it triggers
 * another ADC sample, or terminates the feedback window and, if required, the service command.
 */
//static void feedback_timer_timeout_handler(void *p_context)
//{
//    uint32_t err_code;
//
//    UNUSED_VARIABLE(p_context);
//
//    // Check the counter; if expired, feedback window has finished
//    if (!feedback_window_end)
//    {
//        // Trigger ADC sample
//        NRF_ADC->TASKS_START = 1;
//        err_code = app_timer_start(feedback_timer, APP_TIMER_TICKS(ADC_SAMPLE_INTERVAL, APP_TIMER_PRESCALER), NULL);
//        APP_ERROR_CHECK(err_code);
//    }
//    else
//    {
//        // Buffer is empty in service mode
//        if (function == FUNCTION_WRITE_BYTE || function == FUNCTION_WRITE_BIT)
//        {
//            // We are done now, transmit back result
//            err_code = ble_bluetrack_response_update(&m_bluetrack, acknowledge ? 0x01 : 0x00, value);
//            if (err_code != NRF_SUCCESS &&
//                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            service_command_in_progress = false;
//        }
//        else if (function == FUNCTION_READ_BYTE)
//        {
//            // Last condition is special case to avoid repeating the read byte values for an address mode read
//            if (acknowledge || (read_byte_counter >= READ_BYTE_COUNTER_MAX) || (mode == MODE_ADDRESS && read_byte_counter >= READ_BIT_COUNTER_MAX_ADDRESS))
//            {
//                // We have either received a positive response, or we've exhausted all values, transmit back result
//                err_code = ble_bluetrack_response_update(&m_bluetrack, acknowledge ? 0x01 : 0x00, read_byte_counter);
//                if (err_code != NRF_SUCCESS &&
//                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                    err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//                service_command_in_progress = false;
//            }
//            else
//            {
//                // No response received, continue to try new values
//                read_byte_counter = read_byte_counter + 1;
//                err_code = app_sched_event_put(NULL, 0, execute_service_command);
//                APP_ERROR_CHECK(err_code);
//            }
//        }
//        else if (function == FUNCTION_READ_BIT)
//        {
//            read_bit_value = read_bit_value | ((acknowledge ? 1 : 0) << read_bit_counter);
//            if (read_bit_counter >= READ_BIT_COUNTER_MAX)
//            {
//                // We have finished asking for the value of all bit positions, transmit back result
//                err_code = ble_bluetrack_response_update(&m_bluetrack, 0x01, read_bit_value);
//                if (err_code != NRF_SUCCESS &&
//                    err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//                    err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//                service_command_in_progress = false;
//            }
//            else
//            {
//                // Ask for next bit position
//                read_bit_counter = read_bit_counter + 1;
//                err_code = app_sched_event_put(NULL, 0, execute_service_command);
//                APP_ERROR_CHECK(err_code);
//            }
//        }
//        
//        // Lower feedback flag
//        feedback_in_progress = false;
//    }
//}


/**@brief Function for handling the output application timer expiry.
 *
 * @details Output application timer runs continuously. Every time it completes, it checks if it needs to actuate a relay output by looking at the flags. It passes the value of
 * the relay output to itself so it can clear it on expiry. Flags are searched independent of raise order. Only one relay is ever actuated at a time to protect the power supply.
 */
//static void output_timer_timeout_handler(void *p_context)
//{
//    uint32_t * index = (uint32_t*)p_context;
//    uint32_t i, err_code;
//    bool enqueued_output = false;
//
//    // Turn off output if one was actuated
//    if (index != NULL)
//    {
//        nrf_gpio_pin_clear(*index);
//    }
//
//    // Look for an enqueued output and start timer if we find one
//    for (i = 0; i < (N_OUTPUTS); i++)
//    {
//        if (output_flags[i])
//        {
//            enqueued_output = true;
//
//            // Lower flag
//            output_flags[i] = false;
//
//            // Turn on output
//            nrf_gpio_pin_set(output_index[i]);
//
//            // Start timer.
//            err_code = app_timer_start(output_timer, APP_TIMER_TICKS(ACTUATION_INTERVAL, APP_TIMER_PRESCALER), &(output_index[i]));
//            APP_ERROR_CHECK(err_code);
//
//            // Only one relay is to be actuated at at time to protect the power supply.
//            break;
//        }
//    }
//
//    if (!enqueued_output)
//    {
//        // Start timer with no output actuation
//        err_code = app_timer_start(output_timer, APP_TIMER_TICKS(ACTUATION_INTERVAL, APP_TIMER_PRESCALER), NULL);
//        APP_ERROR_CHECK(err_code);
//    }
//#ifdef DEBUG
//    err_code = app_uart_put_string("Output timer fired\n\r");
//    APP_ERROR_CHECK(err_code);
//#endif
//}


/**@brief Function for initialising the GPIOTE module.
 */
static void gpiote_init(void)
{
    ret_code_t                  err_code;
    uint32_t                    i;
    nrf_drv_gpiote_in_config_t  config_in =               GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    nrf_drv_gpiote_out_config_t config_out_toggle_false = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);   
    nrf_drv_gpiote_out_config_t config_out_simple_false = GPIOTE_CONFIG_OUT_SIMPLE(false);
    nrf_drv_gpiote_out_config_t config_out_simple_true =  GPIOTE_CONFIG_OUT_SIMPLE(true);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // Initialise main DCC output - under PPI control
    err_code = nrf_drv_gpiote_out_init(MAIN_DCC_PIN_NO, &config_out_toggle_false);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_task_enable(MAIN_DCC_PIN_NO);

    // Initialise brake output (start with brake on) - under CPU control
    err_code = nrf_drv_gpiote_out_init(BRAKE_N_PIN_NO, &config_out_simple_false);
    APP_ERROR_CHECK(err_code);

    // Initialise relay outputs (start not actuated) - under CPU control
    for (i =0; i < (N_OUTPUTS); i++)
    {
        err_code = nrf_drv_gpiote_out_init(output_index[i], &config_out_simple_false);
        APP_ERROR_CHECK(err_code);
    }

    // Initialise sync output (start with no SYNC indicated) - under CPU control
    err_code = nrf_drv_gpiote_out_init(SYNC_PIN_NO, &config_out_simple_false);
    APP_ERROR_CHECK(err_code);     

    // Initialise stop LED (start stopped) - under CPU control
    err_code = nrf_drv_gpiote_out_init(STOP_LED_PIN_NO, &config_out_simple_true);
    APP_ERROR_CHECK(err_code);   

    // Initialise programming DCC output - under PPI control
    err_code = nrf_drv_gpiote_out_init(PROG_DCC_PIN_NO, &config_out_toggle_false);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_task_enable(PROG_DCC_PIN_NO);

    // Initialise thermal input - generates GPIOTE event
    config_in.pull = NRF_GPIO_PIN_PULLUP;
    err_code = nrf_drv_gpiote_in_init(THERMAL_N_PIN_NO, &config_in, thermal_n_hi_to_lo_handler);
    APP_ERROR_CHECK(err_code);

    // Initialise overload/overtemperature LED (start with no overload/overtemperature) - under CPU control
    err_code = nrf_drv_gpiote_out_init(OVER_LED_PIN_NO, &config_out_simple_false);
    APP_ERROR_CHECK(err_code);

    // Initialise BLE LED (start off) - under PPI control
    err_code = nrf_drv_gpiote_out_init(BLE_LED_PIN_NO, &config_out_toggle_false);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_task_enable(BLE_LED_PIN_NO);

    // Initialise programming LED (start not in programming mode) - under CPU control
    err_code = nrf_drv_gpiote_out_init(PROG_LED_PIN_NO, &config_out_simple_false);
    APP_ERROR_CHECK(err_code);

    // Enable the thermal interrupt only after all pins are conifigured
    nrf_drv_gpiote_in_event_enable(THERMAL_N_PIN_NO, true);

    // Check if thermal input is already low, and if so disable DCC
    if (!nrf_drv_gpiote_in_is_set(THERMAL_N_PIN_NO))
    {
        //disable_DCC(ERROR_CODE_OVERTEMPERATURE);
    }
}


/**@brief Function for initialising timers.
 *
 * @details Initializes the application timer, as well as TIMER1 and TIMER2.
 */
static void timers_init(void)
{
    ret_code_t             err_code;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;

    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    // Initialise DCC_DATA timer (initially outputs 1s)
    err_code = nrf_drv_timer_init(&TIMER_DCC_DATA, &timer_cfg, timer_dcc_data_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_extended_compare(&TIMER_DCC_DATA, NRF_TIMER_CC_CHANNEL0, nrf_drv_timer_us_to_ticks(&TIMER_DCC_DATA, DCC_ONE_TIME_US), NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    // Initialise BLE_LED timer
    err_code = nrf_drv_timer_init(&TIMER_BLE_LED, &timer_cfg, dummy_timer_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_extended_compare(&TIMER_BLE_LED, NRF_TIMER_CC_CHANNEL0, nrf_drv_timer_ms_to_ticks(&TIMER_BLE_LED, BLE_LED_TOGGLE_TIME_MS), NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    // Initialise DCC_CONTINOUS_ONES timer (always outputs 1s)
    err_code = nrf_drv_timer_init(&TIMER_DCC_CONTINOUS_ONES, &timer_cfg, dummy_timer_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_extended_compare(&TIMER_DCC_CONTINOUS_ONES, NRF_TIMER_CC_CHANNEL0, nrf_drv_timer_us_to_ticks(&TIMER_DCC_CONTINOUS_ONES, DCC_ONE_TIME_US), NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    // Initialize timer module
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create output application timer
    //err_code = app_timer_create(&output_timer, APP_TIMER_MODE_SINGLE_SHOT, output_timer_timeout_handler);
    //APP_ERROR_CHECK(err_code);
    // Create feedback application timer
    //err_code = app_timer_create(&feedback_timer, APP_TIMER_MODE_SINGLE_SHOT, feedback_timer_timeout_handler);
    //APP_ERROR_CHECK(err_code);
}


/** @brief Function for initialising the Programmable Peripheral Interconnect (PPI) module.
 *
 * @details PPI is used for BLE LED, DCC Output, and DCC Programming Output.
 */
static void ppi_init(void)
{
    ret_code_t        err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // Allocate, assign, and enable BLE LED PPI channel (note don't need to retain this channel)
    err_code = nrf_drv_ppi_channel_alloc(&ble_led_timer_to_ble_led);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ble_led_timer_to_ble_led, nrf_drv_timer_event_address_get(&TIMER_BLE_LED, NRF_TIMER_EVENT_COMPARE0), nrf_drv_gpiote_out_task_addr_get(BLE_LED_PIN_NO));
    APP_ERROR_CHECK(err_code);
    nrf_drv_ppi_channel_enable(ble_led_timer_to_ble_led);
    APP_ERROR_CHECK(err_code);

    // Allocate, assign, and enable DCC Output PPI channels (note DCC Data is enabled initially)
    err_code = nrf_drv_ppi_channel_alloc(&dcc_data_to_main_dcc);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(dcc_data_to_main_dcc, nrf_drv_timer_event_address_get(&TIMER_DCC_DATA, NRF_TIMER_EVENT_COMPARE0), nrf_drv_gpiote_out_task_addr_get(MAIN_DCC_PIN_NO));
    APP_ERROR_CHECK(err_code);
    nrf_drv_ppi_channel_enable(dcc_data_to_main_dcc);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_alloc(&dcc_continuous_ones_to_main_dcc);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(dcc_continuous_ones_to_main_dcc, nrf_drv_timer_event_address_get(&TIMER_DCC_CONTINOUS_ONES, NRF_TIMER_EVENT_COMPARE0), nrf_drv_gpiote_out_task_addr_get(MAIN_DCC_PIN_NO));
    APP_ERROR_CHECK(err_code);

    // Allocate, assign, and enable Programming DCC Output PPI channels (note DCC Continuous Ones is enabled initially)
    err_code = nrf_drv_ppi_channel_alloc(&dcc_continuous_ones_to_prog_dcc);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(dcc_continuous_ones_to_prog_dcc, nrf_drv_timer_event_address_get(&TIMER_DCC_CONTINOUS_ONES, NRF_TIMER_EVENT_COMPARE0), nrf_drv_gpiote_out_task_addr_get(PROG_DCC_PIN_NO));
    APP_ERROR_CHECK(err_code);
    nrf_drv_ppi_channel_enable(dcc_continuous_ones_to_prog_dcc);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_alloc(&dcc_data_to_prog_dcc);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(dcc_data_to_prog_dcc, nrf_drv_timer_event_address_get(&TIMER_DCC_DATA, NRF_TIMER_EVENT_COMPARE0), nrf_drv_gpiote_out_task_addr_get(PROG_DCC_PIN_NO));
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the LPCOMP module.
 */
//static void lpcomp_init(void)
//{
//    // Select P0.1 as the LPCOMP input (LPCOMP can be used concurrently with the ADC provided the input is the same, as only the PSEL MUX is shared, see https://devzone.nordicsemi.com/question/22244/can-i-really-not-use-lpcomp-and-adc-at-the-same-time/)
//    NRF_LPCOMP->PSEL = LPCOMP_PSEL_PSEL_AnalogInput2;
//    // Set reference to VDD * 7/8
//    // For sense R = 1.74k, the trip points are:
//
//    // +---------------+------------+------------+
//    // | Sensitivity   | VDD = 2.3V | VDD = 3.0V |
//    // +---------------+------------+------------+
//    // | 300uA/A (min) |    3.86A   |    5.03A   |
//    // +---------------+------------+------------+
//    // | 377uA/A (nom) |    3.07A   |    4.00A   |
//    // +---------------+------------+------------+
//    // | 450uA/A (max) |    2.57A   |    3.35A   |
//    // +---------------+------------+------------+
//    
//    NRF_LPCOMP->REFSEL = LPCOMP_REFSEL_REFSEL_SupplySevenEighthsPrescaling;
//    // Allow READY and UP event to trigger an interrupt
//    NRF_LPCOMP->INTENSET = (LPCOMP_INTENSET_UP_Msk | LPCOMP_INTENSET_READY_Msk);
//    // Finally, turn on the comparator
//    NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
//
//    // Register LPCOMP interrupt with softdevice
//    NVIC_SetPriority(LPCOMP_IRQn, LPCOMP_IRQ_PRI);
//    NVIC_ClearPendingIRQ(LPCOMP_IRQn);
//    NVIC_EnableIRQ(LPCOMP_IRQn);
//
//    // Start up the comparator
//    NRF_LPCOMP->TASKS_START = 1;
//}


/**@brief Function for initializing the ADC module.
*/
//static void adc_init(void)
//{
//    // Select P0.1 as the ADC input (ADC can be used concurrently with the LPCOMP provided the input is the same), 1.2V bandgap as reference, no prescaling, and 10 bit resolution
//    NRF_ADC->CONFIG = (
//                       (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) |
//                       (ADC_CONFIG_PSEL_AnalogInput2 << ADC_CONFIG_PSEL_Pos) |
//                       (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
//                       (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos) |
//                       (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos)
//                      );
//    // Allow END event to trigger an interrupt
//    NRF_ADC->INTENSET = ADC_INTENSET_END_Msk;
//    // Finally, turn on the ADC
//    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
//
//    // Register ADC interrupt with softdevice
//    NVIC_SetPriority(ADC_IRQn, ADC_IRQ_PRI);
//    NVIC_ClearPendingIRQ(ADC_IRQn);
//    NVIC_EnableIRQ(ADC_IRQn);
//}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing BlueTrack data structures.
 */
//static void bluetrack_init(void)
//{
//    uint32_t i;
//
//    // Populate index array and initialise flags
//    for(i = 0; i < (N_OUTPUTS); i++)
//    {
//        output_index[i] = i + OUTPUT_OFFSET;
//        output_flags[i] = false;
//    }
//
//    // Clear buffer
//    memset(dcc_command_buffer, 0, sizeof(dcc_command_buffer));
//    
//    // Clear idle memory
//    memset(&idle_packet, 0, sizeof(idle_packet));
//
//    // Set indexes and phases
//    producer_index = 0;
//    consumer_index = 0;
//    phase1_complete = true;
//    phase2_complete = false;
//
//    // We start with no pending speed commands
//    remove_all_speed_commands();
//    active_speed_command_index = 0;
//    
//    // Send ones at the start
//    dcc_output_state = true;
//
//    // We start off with DCC enabled
//    dcc_disabled = false;
//
//    // We start of with feedback not in progress
//    feedback_in_progress = false;
//
//    // We start off with the main track selected
//    programming_track_mode = MAIN;
//    programming_track_state = MAIN;
//    
//    // We start off with no service command pending
//    service_command_pending = false;
//    
//    // We start off with no service command in progress
//    service_command_in_progress = false;
//    
//    // We do not send 20 reset packets and 10 idle packets, as suggested by S-9.2.4
//}


/**@brief Function for starting timers.
*/
static void timers_start(void)
{
    //uint32_t err_code;

    nrf_drv_timer_enable(&TIMER_DCC_DATA);
    nrf_drv_timer_enable(&TIMER_BLE_LED);
    nrf_drv_timer_enable(&TIMER_DCC_CONTINOUS_ONES);

    // Start relay timer
    //err_code = app_timer_start(output_timer, APP_TIMER_TICKS(ACTUATION_INTERVAL, APP_TIMER_PRESCALER), NULL);
    //APP_ERROR_CHECK(err_code);

}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_addr_t          address;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t               err_code;
    ble_advdata_t            advdata;
    ble_advdata_t            srdata;
    ble_gap_adv_params_t     adv_params;
    ble_advdata_manuf_data_t manuf_data;
    uint32_t                 deviceID[2];
    uint8_t                  identifier[12];
    uint32_t                 firmware_version;
    ble_uuid_t adv_uuids[] = {{BLUETRACK_UUID_SERVICE, m_bluetrack.uuid_type}};

    // Retrieve firmware version from UICR, CUSTOMER[0] stores the application version. */
    firmware_version = NRF_UICR->CUSTOMER[0];
    
    // Construct manufacturing data
    manuf_data.company_identifier = DREKKER_DEVELOPMENT_COMPANY_IDENTIFIER;

    deviceID[0] = NRF_FICR->DEVICEID[0];
    deviceID[1] = NRF_FICR->DEVICEID[1];

    NRF_LOG_INFO("deviceID[0] = 0x%x, deviceID[1] = 0x%x", deviceID[0], deviceID[1]);

    // Make everything little endian
    identifier[0] = firmware_version & 0xFF;
    identifier[1] = (firmware_version >> 8) & 0xFF;
    identifier[2] = (firmware_version >> 16) & 0xFF;
    identifier[3] = (firmware_version >> 24) & 0xFF;
    identifier[4] = deviceID[0] & 0xFF;
    identifier[5] = (deviceID[0] >> 8) & 0xFF;
    identifier[6] = (deviceID[0] >> 16) & 0xFF;
    identifier[7] = (deviceID[0] >> 24) & 0xFF;
    identifier[8] = deviceID[1] & 0xFF;
    identifier[9] = (deviceID[1] >> 8) & 0xFF;
    identifier[10] = (deviceID[1] >> 16) & 0xFF;
    identifier[11] = (deviceID[1] >> 24) & 0xFF;

    manuf_data.data.p_data = identifier;
    manuf_data.data.size = sizeof(identifier);

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // Set this as we advertise for an unlimited time
    advdata.p_manuf_specific_data   = &manuf_data;
    
    // Build and set scan response data
    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t           err_code;
    ble_bluetrack_init_t init;
    ble_dis_init_t       dis_init;
    nrf_ble_qwr_init_t   qwr_init = {0};
    ble_srv_utf8_str_t   manufact_name_str = {.length = strlen(MANUFACTURER_NAME),.p_str  = (uint8_t *)MANUFACTURER_NAME};
    ble_srv_utf8_str_t   model_num_str = {.length = strlen(MODEL_NUMBER),.p_str  = (uint8_t *)MODEL_NUMBER}; 

    // Initialise Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Intitialise BlueTrack write handlers
    init.address_write_handler = address_write_handler;
    init.dcc_command_write_handler = dcc_command_write_handler;
    init.programming_track_select_write_handler = programming_track_select_write_handler;
    init.stop_write_handler = stop_write_handler;
    init.service_command_write_handler = service_command_write_handler;

    err_code = ble_bluetrack_init(&m_bluetrack, &init);
    APP_ERROR_CHECK(err_code);

    // Initialise DIS data
    memset(&dis_init, 0, sizeof(dis_init));
    dis_init.manufact_name_str = manufact_name_str;
    dis_init.model_num_str = model_num_str;
    dis_init.dis_char_rd_sec = SEC_OPEN;
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t               err_code;
    ble_conn_params_init_t   cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t             err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t                       err_code;
    ble_gatts_evt_write_t const * p_evt_write;
    uint8_t const * data;
//    static ble_gap_evt_auth_status_t m_auth_status;
//    ble_gap_enc_info_t *             p_enc_info;
//    uint16_t                         m_programming_track_select_len = PROGRAMMING_TRACK_SELECT_CHAR_SIZE;
//    uint8_t                          m_programming_track_select_value = 0x00;
//    uint16_t                         m_stop_len = STOP_CHAR_SIZE;
//    uint8_t                          m_stop_value = 0x01;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");

            // Disable timer control of BLE LED
            nrf_drv_ppi_channel_disable(ble_led_timer_to_ble_led);
            APP_ERROR_CHECK(err_code);

            // Turn on BLE LED
            nrf_drv_gpiote_out_task_force(BLE_LED_PIN_NO, true);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
      
            // Enable timer control of BLE LED
            nrf_drv_ppi_channel_enable(ble_led_timer_to_ble_led);
            APP_ERROR_CHECK(err_code);
            
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

//            // Remove all pending periodic speed commands
//            remove_all_speed_commands();
            
//            // Reset programming track selection and characteristic
//            programming_track_mode = MAIN;
//            err_code = sd_ble_gatts_value_set(m_bluetrack.programming_track_select_char_handles.value_handle, 0, &m_programming_track_select_len, &m_programming_track_select_value);
//            APP_ERROR_CHECK(err_code);

//            // Reset stop selection and characteristic
//            nrf_gpio_pin_clear(BRAKE_PIN_NO);
//            nrf_gpio_pin_set(STOP_LED_PIN_NO);
//            err_code = sd_ble_gatts_value_set(m_bluetrack.stop_char_handles.value_handle, 0, &m_stop_len, &m_stop_value);
//            APP_ERROR_CHECK(err_code);

            // Restart advertising
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_WRITE:

            p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
            data = p_evt_write->data;
            // Empirically, Service Changed CCCD has handle 0xD, and is always Indicated (value is 0x2)
            // Note data is little endian
            if((p_evt_write->handle == 0xD) && (p_evt_write->len == 2) && (data[0] == 0x2)) {
                NRF_LOG_INFO("Service Changed ready to Indicate");
                err_code = gscm_service_changed_ind_send(m_conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize
    gpiote_init();
    timers_init();
    log_init();
    power_management_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    ppi_init();
//    lpcomp_init();
//    adc_init();
//    bluetrack_init();

    // Start execution
    NRF_LOG_INFO("Bluetrack started.");
    timers_start();
    advertising_start();

    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        idle_state_handle();
    }
}

/**
 * @}
 */
