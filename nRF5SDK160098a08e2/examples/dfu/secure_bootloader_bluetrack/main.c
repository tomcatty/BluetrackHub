/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define MAIN_DCC_PIN_NO                 2
#define BRAKE_N_PIN_NO                  26
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
#define RELAY_15_PIN_NO                 21
#define STOP_LED_PIN_NO                 25
#define PROG_DCC_PIN_NO                 27                            
#define ERROR_LED_PIN_NO                29
#define BLE_LED_PIN_NO                  30
#define PROG_LED_PIN_NO                 31

#define N_OUTPUTS                       15

static uint8_t                          output_pin[N_OUTPUTS] = {RELAY_1_PIN_NO,
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
                                                                 RELAY_14_PIN_NO,
                                                                 RELAY_15_PIN_NO};

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

/**@brief Function for initialising the GPIOs.
 */
static void gpio_init(void)
{
    uint8_t i;

    // Initialise stop LED (start off)
    nrf_gpio_cfg_output(STOP_LED_PIN_NO);
    nrf_gpio_pin_clear(STOP_LED_PIN_NO);

    // Initialise error LED (start off)
    nrf_gpio_cfg_output(ERROR_LED_PIN_NO);
    nrf_gpio_pin_clear(ERROR_LED_PIN_NO);

    // Initialise BLE LED (start off)
    nrf_gpio_cfg_output(BLE_LED_PIN_NO);
    nrf_gpio_pin_clear(BLE_LED_PIN_NO);

    // Initialise programming LED (start off)
    nrf_gpio_cfg_output(PROG_LED_PIN_NO);
    nrf_gpio_pin_clear(PROG_LED_PIN_NO);

    // Initialise main DCC output (always off)
    nrf_gpio_cfg_output(MAIN_DCC_PIN_NO);
    nrf_gpio_pin_clear(MAIN_DCC_PIN_NO);

    // Initialise brake output (always on)
    nrf_gpio_cfg_output(BRAKE_N_PIN_NO);
    nrf_gpio_pin_set(BRAKE_N_PIN_NO);

    // Initialise relay outputs (always off)
    for (i = 0; i < (N_OUTPUTS); i++)
    {
        nrf_gpio_cfg_output(output_pin[i]);
        nrf_gpio_pin_clear(output_pin[i]);
    }

    // Initialise programming DCC output (always off)
    nrf_gpio_cfg_output(PROG_DCC_PIN_NO);
    nrf_gpio_pin_clear(PROG_DCC_PIN_NO);
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_INITIALIZED:        /**< Starting DFU. */
            gpio_init();
            nrf_gpio_pin_set(PROG_LED_PIN_NO);
            nrf_gpio_pin_set(STOP_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:    /**< Transport activated (e.g. BLE connected, USB plugged in). */
            nrf_gpio_pin_set(BLE_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_TRANSPORT_DEACTIVATED:  /**< Transport deactivated (e.g. BLE disconnected, USB plugged out). */
            nrf_gpio_pin_clear(BLE_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_DFU_STARTED:            /**< DFU process started. */
            nrf_gpio_pin_clear(STOP_LED_PIN_NO);
            nrf_gpio_pin_clear(ERROR_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_OBJECT_RECEIVED:        /**< A DFU data object has been received. */
            nrf_gpio_pin_toggle(STOP_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_DFU_FAILED:             /**< DFU process has failed, been interrupted, or hung. */
            nrf_gpio_pin_set(ERROR_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_DFU_COMPLETED:          /**< DFU process completed. */
            nrf_gpio_pin_set(STOP_LED_PIN_NO);
            break;
        case NRF_DFU_EVT_DFU_ABORTED:            /**< DFU process aborted. */
            nrf_gpio_pin_set(STOP_LED_PIN_NO);
            break;
        default:
            break;
    }
}


/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;

    // Must happen before flash protection is applied, since it edits a protected page.
    nrf_bootloader_mbr_addrs_populate();

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Inside main");

    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    NRF_LOG_FLUSH();

    NRF_LOG_ERROR("After main, should never be reached.");
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK_BOOL(false);
}

/**
 * @}
 */
