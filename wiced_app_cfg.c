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

#include <wiced_app_cfg.h>
#include <wiced_bt_audio.h>
#include <wiced_bt_avdt.h>
#include <wiced_bt_avrc.h>
#include <wiced_bt_avrc_defs.h>
#include <wiced_bt_avrc_tg.h>
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
#include <wiced_bt_a2dp_sink.h>
#include <wiced_bt_a2d_sbc.h>
#endif
#include <wiced_bt_ble.h>
#include <wiced_bt_dev.h>
#include <wiced_bt_gatt.h>
#include <wiced_bt_sdp.h>
#include "wiced_app.h"
#include "wiced_app_cfg.h"
#include "cycfg_gatt_db.h"

#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#endif

/* AAC CODEC */
#if (WICED_A2DP_EXT_CODEC == WICED_TRUE)
extern wiced_codec_interface_functions_t AAC_codec_function_table;
#endif

/* If APP_AVRC_TRACK_INFO_SUPPORTED, APP_AVRC_PLAY_STATUS_SUPPORTED or APP_AVRC_SETTING_CHANGE_SUPPORTED are supported, set the AVRC profile
 version as 1.3, else set it to 1.0 */
#if (defined(APP_AVRC_TRACK_INFO_SUPPORTED) || defined(APP_AVRC_PLAY_STATUS_SUPPORTED) || defined(APP_AVRC_SETTING_CHANGE_SUPPORTED))
#define AVRC_PROFILE_VER  AVRC_REV_1_5
#else
#define AVRC_PROFILE_VER  AVRC_REV_1_0
#endif

/*
 * Definitions
 */
// SDP Record handle for AVDT Source
#define HANDLE_AVDT_SOURCE                      0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for HFP Audio Gateway
#define HANDLE_HFP_AG                           0x10004
// SDP Record handle for HFP Hands-Free Unit
#define HANDLE_HFP_HF                           0x10005
// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10006
// SDP Record handle for PNP (Device Information)
#define HANDLE_PNP                              0x10007

#if defined(CYW43012C0) || defined(CYW20819A1)
#define AUDIO_TX_BUFFER_SIZE        0x2000
#else
#define AUDIO_TX_BUFFER_SIZE        11000
#endif

/* It needs 14728 bytes for HFP(mSBC use mainly) and 14148 bytes for A2DP(jitter buffer use mainly) */
#define AUDIO_BUF_SIZE_MAIN                 (15 * 1024)

#ifdef A2DP_SINK_AAC_ENABLED
/* For AAC, audio codec memory requires 21248 bytes and sample buffer required 2 * 4 1024 bytes */
#define AUDIO_BUF_SIZE_CODEC                (21248 + (2 * 4 * 1024))
#else
/* SBC audio codec memory requires 6908 bytes */
#define AUDIO_BUF_SIZE_CODEC                6908
#endif

#define AUDIO_CODEC_BUFFER_SIZE             (AUDIO_BUF_SIZE_MAIN + AUDIO_BUF_SIZE_CODEC)

const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name                         = (uint8_t*)app_gap_device_name,                               /**< Local device name (NULL terminated). Use same as configurator generated string.*/
    .device_class                        = {0x20, 0x07, 0x04},                                         /**< Local device class */
    .security_requirement_mask           = BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT, /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 3,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                                                                 /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                                                    /* LE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< LE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 60,                                                         /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = 112,                                                        /**< Minimum connection interval */
        .conn_max_interval               = 128,                                                        /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                                                                  /* LE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */

        .low_duty_min_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                                                        /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = WICED_MAX_LE_CLIENT_CONN,                                    /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 1,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 243 - 5,                                                     /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 243,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                                                      /* RFCOMM configuration */
    {
        .max_links                      = WICED_BT_RFCOMM_MAX_CONN,                                    /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = WICED_BT_RFCOMM_MAX_CONN,                                    /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                                                               /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 1                                                            /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 1,                                                           /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = 3                                                            /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                                                        /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 1                                                            /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 11,                                                          /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
    /* LE Filter Accept List size */
    .ble_filter_accept_list_size                = 2,                                                           /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    .default_ble_power_level            = 12                                                           /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
#define SLEN_A2DP_SRC   (56 + 2)
#else
#define SLEN_A2DP_SRC   0
#endif
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
#define SLEN_A2DP_SNK   (77 + 2)
#else
#define SLEN_A2DP_SNK   0
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
#define SLEN_AVRC_TG    (56 + 2)
#else
#define SLEN_AVRC_TG    0
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
#define SLEN_AVRC_CT    (59 + 2)
#else
#define SLEN_AVRC_CT    0
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#define SLEN_HFP_AG     (60 + 2)
#else
#define SLEN_HFP_AG     0
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#define SLEN_HFP_HF     (75 + 2)
#else
#define SLEN_HFP_HF     0
#endif
#define SLEN_PNP        (69 + 2)

#define SLEN            (SLEN_A2DP_SRC + SLEN_A2DP_SNK + SLEN_AVRC_TG + SLEN_AVRC_CT + SLEN_HFP_AG + SLEN_HFP_HF + SLEN_PNP)

/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t wiced_app_cfg_sdp_record[] =
{
    // length is the sum of all records
    SDP_ATTR_SEQUENCE_2(SLEN),

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    // SDP Record for AVDT Source
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SOURCE),                         // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SOURCE),                     // 8 bytes
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),   // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),                         // 3 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),                       // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2 bytes
            SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),     // 3 bytes
                SDP_ATTR_VALUE_UINT2(0x103),                                // 3 bytes
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0001),
#endif

#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x100),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_PROFILE_VER),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT1),
#endif

#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT2),
#endif

#ifdef WICED_APP_HFP_AG_INCLUDED
    // SDP record for HFP AG ( total length of record: 62 )
    SDP_ATTR_SEQUENCE_1( 60 ),                                              // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE( HANDLE_HFP_AG ),                            // 8 byte ( handle=0x10004 )
        SDP_ATTR_ID( ATTR_ID_SERVICE_CLASS_ID_LIST ),                       // 3 bytes
            SDP_ATTR_SEQUENCE_1( 6 ),                                       // 2 bytes
                SDP_ATTR_UUID16( UUID_SERVCLASS_AG_HANDSFREE ),             // 3 bytes ServiceClass0 UUID_SERVCLASS_AG_HANDSFREE
                SDP_ATTR_UUID16( UUID_SERVCLASS_GENERIC_AUDIO ),            // 3 bytes ServiceClass1 UUID_SERVCLASS_GENERIC_AUDIO
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 1 ),                            // 17 bytes ( SCN=1 )
        SDP_ATTR_PROFILE_DESC_LIST( UUID_SERVCLASS_AG_HANDSFREE, 0x0108 ),  // 13 bytes UUID_SERVCLASS_HF_HANDSFREE, version 0x0108
        SDP_ATTR_UINT1(ATTR_ID_NETWORK, 0x00),                                  // 5 byte
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES,  AG_SUPPORTED_FEATURES_ATT), //6 bytes
#endif

#ifdef WICED_APP_HFP_HF_INCLUDED
    // SDP Record for Hands-Free Unit
    SDP_ATTR_SEQUENCE_1(75),
        SDP_ATTR_RECORD_HANDLE( HANDLE_HFP_HF ),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( HANDS_FREE_SCN ),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                SDP_ATTR_VALUE_UINT2(0x0107),
        SDP_ATTR_SERVICE_NAME(15),
            'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, SUPPORTED_FEATURES_ATT),
#endif

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
    // SDP Record for A2DP Sink (total = 77 + 2 = 79)
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_SERVICE_NAME(16),
                'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),
#endif

    // SDP record Device ID (total = 69 + 2 = 71)
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HANDLE_PNP),                                 // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0201),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),// 6
};

/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       16   },      /* Small Buffer Pool */
    { 360,      4   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
#ifdef CYW20819A1
    { 360,      9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     3   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#else
    { 1024,     9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     5   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
#endif
};

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
    },

#ifdef A2DP_SINK_AAC_ENABLED
    {
        .codec_id = WICED_BT_A2DP_CODEC_M24,
        .cie =
        {
            .m24 =
            {
                (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /*samp_freq */
                (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
            }
        }
    }
#endif
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
#ifdef A2DP_SINK_ENABLE_CONTENT_PROTECTION
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_PROTECT | WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,    /* feature mask */
#else
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                      /* feature mask */
#endif
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                                           /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#ifdef A2DP_SINK_AAC_ENABLED
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
        .codec_functions = &AAC_codec_function_table,
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = 0,
#endif
    },
};
#endif /* WICED_APP_AUDIO_SNK_INCLUDED */

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config = {
    .role                             =   WICED_AUDIO_SOURCE_ROLE,  /* TODO: SOURCE | SINK, no impact */
    .audio_tx_buffer_size             =   AUDIO_TX_BUFFER_SIZE,
    .audio_codec_buffer_size          =   AUDIO_CODEC_BUFFER_SIZE
#if defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1)
    ,.audio_tx_buffer_watermark_level =   50
#endif
};

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}

/*
 * wiced_app_cfg_sdp_record_get
 */
uint8_t *wiced_app_cfg_sdp_record_get(void)
{
    return (uint8_t *)wiced_app_cfg_sdp_record;
}

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(wiced_app_cfg_sdp_record);
}

#ifdef WICED_APP_AUDIO_SNK_INCLUDED
wiced_bt_a2dp_config_data_t wiced_app_cfg_a2dp_config_data =
{
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                  /* feature mask */
    .codec_capabilities =
    {
        .count = 0,
        .info  = NULL,
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = NULL,
    }
};
#endif /* WICED_APP_AUDIO_SNK_INCLUDED */

uint8_t wiced_app_cfg_avrc_ct_supported_events[] =
{
    [AVRC_EVT_PLAY_STATUS_CHANGE] = 1,
    [AVRC_EVT_TRACK_CHANGE] = 1,
    [AVRC_EVT_TRACK_REACHED_END] = 1,
    [AVRC_EVT_TRACK_REACHED_START] = 1,
    [AVRC_EVT_PLAY_POS_CHANGED] = 1,
    [AVRC_EVT_BATTERY_STATUS_CHANGE] = 0,
    [AVRC_EVT_SYSTEM_STATUS_CHANGE] = 0,
    [AVRC_EVT_APP_SETTING_CHANGE] = 1,
    [AVRC_EVT_NOW_PLAYING_CHANGE] = 0,
    [AVRC_EVT_AVAL_PLAYERS_CHANGE] = 0,
    [AVRC_EVT_ADDR_PLAYER_CHANGE] = 0,
    [AVRC_EVT_UIDS_CHANGE] = 0,
    [AVRC_EVT_VOLUME_CHANGE] = 1,
};
