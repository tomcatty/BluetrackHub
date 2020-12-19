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
#include "nrf_crypto.h"
#include "nrf_crypto_shared.h"

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

#define FICR_DEVICEID_LEN               8

__ALIGN(4) extern const uint8_t pk[64];

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
    uint8_t  pk_copy[sizeof(pk)];
    uint8_t  *p_signature;
    uint8_t  data[FICR_DEVICEID_LEN];
    nrf_crypto_ecc_public_key_t             public_key;
    size_t hash_len = NRF_CRYPTO_HASH_SIZE_SHA256;
    nrf_crypto_hash_context_t         hash_context   = {0};
    nrf_crypto_ecdsa_verify_context_t verify_context = {0};
    nrf_crypto_hash_sha256_digest_t              hash;
    nrf_crypto_ecdsa_secp256r1_signature_t       signature;

    uint8_t dummy_signature[NRF_CRYPTO_ECDSA_SECP256R1_SIGNATURE_SIZE] =
    {
        0xe1, 0x8e, 0xbd, 0xab, 0x9f, 0x9e, 0xe8, 0x25, 0x8b, 0xc8, 0xbe, 0xa4, 0x27, 0xdf, 0x68, 0x9e, 0x52, 0x5c, 0xd9, 0x49, 0x7a, 0xf7, 0x88, 0x73, 0x91, 0x8f, 0x69, 0x51, 0x04, 0x62, 0xea, 0xb8,
        0xd1, 0x7b, 0xbc, 0x31, 0x9a, 0x2d, 0x96, 0x88, 0x1a, 0xbb, 0xcb, 0xa4, 0xf7, 0x87, 0xea, 0x57, 0x81, 0xc4, 0x97, 0xcc, 0x1f, 0x9b, 0xe6, 0x10, 0x77, 0x00, 0x67, 0x74, 0x88, 0x36, 0xfa, 0x77
    };

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

    NRF_LOG_INFO("Checking hardware signature")
 
    ret_val = nrf_crypto_init();
    APP_ERROR_CHECK(ret_val);

    // Convert public key to big-endian format for use in nrf_crypto.
    nrf_crypto_internal_double_swap_endian(pk_copy, pk, sizeof(pk) / 2);

    ret_val = nrf_crypto_ecc_public_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info,
                                                  &public_key,
                                                  pk_copy,
                                                  sizeof(pk));
    APP_ERROR_CHECK(ret_val);

    // Data is FICR DEVICEID
    memcpy(data, (uint8_t*)&(NRF_FICR->DEVICEID[0]), FICR_DEVICEID_LEN);

    NRF_LOG_INFO("Calculating FICR DEVICEID hash (len: %d)", FICR_DEVICEID_LEN);
    ret_val = nrf_crypto_hash_calculate(&hash_context,
                                         &g_nrf_crypto_hash_sha256_info,
                                         data,
                                         FICR_DEVICEID_LEN,
                                         hash,
                                         &hash_len);
    APP_ERROR_CHECK(ret_val);

    // Signature starts in UICR[1] and is big-endian for nrf_crypto use
    memcpy(signature, dummy_signature, NRF_CRYPTO_ECDSA_SECP256R1_SIGNATURE_SIZE);//&(NRF_UICR->CUSTOMER[1]), NRF_CRYPTO_ECDSA_SECP256R1_SIGNATURE_SIZE);

    // Calculate the signature.
    NRF_LOG_INFO("Verify hardware signature");
    ret_val = nrf_crypto_ecdsa_verify(&verify_context,
                                       &public_key,
                                       hash,
                                       hash_len,
                                       signature,
                                       NRF_CRYPTO_ECDSA_SECP256R1_SIGNATURE_SIZE);
    if (ret_val != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Signature failed (err_code: 0x%x)", ret_val);
        NRF_LOG_DEBUG("Signature:");
        NRF_LOG_HEXDUMP_DEBUG(signature, sizeof(signature));
        NRF_LOG_DEBUG("Hash:");
        NRF_LOG_HEXDUMP_DEBUG(hash, hash_len);
        NRF_LOG_DEBUG("Data:");
        NRF_LOG_HEXDUMP_DEBUG(data, FICR_DEVICEID_LEN);
        NRF_LOG_DEBUG("Public Key:");
        NRF_LOG_HEXDUMP_DEBUG(pk_copy, sizeof(pk_copy));
        NRF_LOG_FLUSH();

        APP_ERROR_CHECK(ret_val);
    }

    NRF_LOG_INFO("Hardware signature verified");

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
