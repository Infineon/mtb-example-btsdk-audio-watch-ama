/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * Runtime Bluetooth stack configuration parameters
 *
 */

#pragma once

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
#include <wiced_bt_a2dp_sink.h>
#endif
#include <wiced_bt_cfg.h>
#include <wiced_bt_audio.h>

#ifdef WICED_APP_HFP_AG_INCLUDED
#define HFP_AG_RFCOMM_CONN_NUM    2
#else
#define HFP_AG_RFCOMM_CONN_NUM    0
#endif

#ifdef WICED_APP_HFP_HF_INCLUDED
#define HFP_HF_RFCOMM_CONN_NUM    1
#else
#define HFP_HF_RFCOMM_CONN_NUM    0
#endif

#define WICED_BT_RFCOMM_MAX_CONN    (HFP_AG_RFCOMM_CONN_NUM + HFP_HF_RFCOMM_CONN_NUM)

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

#ifndef BTSTACK_VER
//const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void);
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];
#endif
extern const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config;
extern int wiced_app_cfg_get_num_buf_pools(void);

//uint8_t *wiced_app_cfg_sdp_record_get(void);
extern const uint8_t wiced_app_cfg_sdp_record[];
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
extern wiced_bt_a2dp_config_data_t wiced_app_cfg_a2dp_config_data;
#endif
extern uint8_t wiced_app_cfg_avrc_ct_supported_events[];
