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
 *  watch.c
 *
 *  Watch Sample Application for 20XXX devices.
 *
 *  This app demonstrates Bluetooth A2DP source, AVRCP Controller/Target, Apple Media Service (AMS),
 *  Apple Notification Center Service (ANCS), and HFP Audio Gateway/Handsfree Unit.
 *  Features demonstrated
 *   - WICED Bluetooth A2DP Source APIs
 *   - WICED Bluetooth AVRCP (Controller/Target) APIs
 *   - WICED Bluetooth GATT APIs
 *   - Apple Media Service and Apple Notification Client Services (AMS and ANCS)
 *   - Handling of the UART WICED protocol
 *   - SDP and GATT descriptor/attribute configuration
 *   - HFP Audio Gateway role
 *   - HFP Handsfree Unit role
 *
 *  Instructions
 *  ------------
 *  To demonstrate the app, follow these steps -
 *
 *  1. Build and download the application to the WICED board.
 *  2. By default sleep in enabled in 20706A2, 20819A1, 20719B1, 20721B1 chips (*refer : Note).
 *  3. Open the Client Control application and open the port for WICED HCI for the device.
 *     Default baud rate configured in the application is 3M.
 *  4. Use Client Control application to send various commands mentioned below.
 *  5. Run the BTSpy program to view protocol and application traces.
 *
 *  Note :
 *   For 20706A2 we are allowing normal PMU sleep.Therefore,the transport will be connected by default.So,no need to wake device.
 *   In other chips 207xx, 208xx please wake the device using configured wake pin. (Check  hci_control_sleep_config.device_wake_gpio_num).
 *
 *  See "Client Control" and "BT Spy" in chip-specifc readme.txt for more information about these apps.
 *
 *  BR/EDR Audio Source and AVRC Target:
 *  - The Watch app can demonstrate how use to BR/EDR Audio Source and AVRC TG profiles.
 *  - Use buttons in AV Source tab.
 *  - To play sine wave sample, set the audio frequency to desired value (48kHz, 44.1kHz, etc.)
 *    and select the Media type as 'Sine Wave' in UI. In this case, built-in sine wave audio is played.
 *  - To play music from .wav file, select the Media type as File, browse and select a .wav file.
 *    In this case, audio for .wav file is routed over WICED HCI UART to the WICED board.
 *  - Put an audio sink device such as Bluetooth headphone/speaker in pairable mode.
 *  - Click on "Start" button for "BR/EDR Discovery" combo box to find the audio sink device.
 *  - Select the peer device in the BR/EDR Discovery combo box.
 *  - Click "Connect" button under AV Source tab.
 *  - Click "Start Streaming" button. Music will start playing on peer device.
 *  - The watch app uses AVRCP Target role. Once connected to headset/speaker,
 *    the app can send notifications for play status change (Play, Pause, Stop) and
 *    setting change (Repeat, Shuffle) to peer AVRCP controller (such as headset/speaker).
 *    Note: the songs shown in the AVRC TG UI and some settings such Repeat/Shuffle are for testing
 *    AVRC commands only, do not indicate the actual media played and will not change the media played.
 *
 *  BR/EDR AVRCP Controller:
 *  - The Watch app can demonstrate how use to AVRC CT profile.
 *  - Disconnect all devices if any connected.
 *  - Make an audio source device such as iPhone discoverable/pairable from Bluetooth Settings UI on phone.
 *  - Using "BR/EDR Discovery" "Start" button, search and select the device.
 *  - Use buttons in AVRC CT tab. Click Connect button and accept pairing.
 *  - Play music on audio source device and control the music via buttons in AVRC CT tab.
 *  - In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
 *  - Absolute volume change can be done via the drop down Volume or Vol Up/Down buttons.
 *  - Note that iPhone does does not support Vol buttons.
 *  - Note that music will continue to play on audio source device.
 *
 *  iOS ANCS and AMS GATT Services:
 *  - The Watch app can demonstrate how to use AMS and ANCS iOS services as below.
 *  - Disconnect all devices if any connected.
 *  - Select Pairable if it not checked.
 *  - Click on the "Start Adverts" button in GATT tab.
 *  - From the iPhone app such as 'LightBlue', find and connect to 'Watch' app.
 *  - Allow pairing with the iPhone.
 *  - AMS:
 *    - Play media on the iPhone.
 *    - Use buttons in AVRC CT tab to control the music.
 *    - Note that music will continue to play on iPhone.
 *  - ANCS:
 *    - Incoming calls and messages to the iPhone will be displayed on the ANCS buttons.
 *    - Make an incoming call to your iPhone. See notification displayed on UI to accept
 *      or reject the call. Send SMS to your iPhone to see notification. Similarly missed
 *      call notifications are seen.
 *
 *  LE Client:
 *  - The Watch app can demonstrate LE Client functionality as below.
 *  - Make sure there is a Bluetooth device with GATT services that is advertising. For example use app
 *    such as 'LightBlue' on your phone and create a 'Virtual Peripheral' such as 'Blood Pressure'
 *  - To find GATT devices: Click on the "Start" button for "LE Discovery" combo box.
 *    Click on "Stop" button to end discovery.
 *  - To connect LE device: Choose device from the "LE Discovery" drop down combo box and
 *    click "Connect" button.
 *  - To discover services: Click on the "Discover Services" button
 *  - To discover characteristics: Enter the handles in the edit box and click
 *    on "Discover Characteristics"
 *  - To discover descriptors: Enter the handles in the edit box and click on
 *    "Discover Characteristics"
 *  - Enter the Handle and Hex Value to write to the remote device using buttons
 *     "Write" : Write hex value to remote handle
 *     "Write no rsp" : Write hex value without response to remote handle
 *     "Value Notify" : Write notification value to the remote handle
 *     "Value Indicate" : Write indication value to the remote handle
 *
 *  HFP Handsfree Unit:
 *  - Targets CYW920721M2EVK-01 CYW920721M2EVK-02 CYW920721M2EVB-03 and CYW9M2BASE-43012BT support
 *    HFP Handsfree Unit as default.
 *  - To create handsfree connection with remote Audio Gateway (AG) device (such as mobile phone), using
 *    ClientControl and choose the Bluetooth address of the remote AG device from the BR/EDR combo box.
 *    Click "Connect" button under HF tab.
 *  - OR Put the device in discoverable and connectable mode and search for the device from AG device and connect
 *  - The following HF operations can be performed using the ClientControl HF tab
 *       Connect / Disconnect HF or SCO connection
 *       Answer / Hang-up the call
 *       Dial / Redial the number
 *       Control Held call (ex. hold call, release all held calls, etc.)
 *       Mic / Speaker gain control
 *
 *  SPI Transport:
 *  - Update the functionality of WICED_P06 from WICED_GPIO to WICED_SPI_1_SLAVE_READY in
 *    platform_gpio_pins[] to get SPI transport working for 20719B2
 */

/*******************************************************************************
 *                               Includes
 ******************************************************************************/
#ifdef AMA_ENABLED
#include <ama_hci_control.h>
#endif
#include <string.h>
#ifdef WICED_APP_AMS_INCLUDED
#include <wiced_bt_ams.h>
#endif
#ifdef WICED_APP_ANCS_INCLUDED
#include <wiced_bt_ancs.h>
#endif
#include <wiced_bt_avrc.h>
#include <wiced_bt_avrc_defs.h>
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
#include <wiced_bt_avrc_tg.h>
#endif
#include <wiced_bt_ble.h>
#include <wiced_bt_cfg.h>
#include <wiced_bt_gatt.h>
#include <wiced_bt_sdp.h>
#include <wiced_hal_puart.h>
#include <wiced_hal_gpio.h>
#ifdef CYW43012C0
#include <wiced_hal_watchdog.h>
#else
#include <wiced_hal_wdog.h>
#endif
#include <wiced_memory.h>
#include <wiced_platform.h>
#include <wiced_timer.h>
#include <wiced_transport.h>
#include <wiced_bt_stack.h>
#ifdef CYW9BT_AUDIO
#include <wiced_audio_manager.h>
#endif
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
#include <wiced_bt_app_common.h>
#endif
#ifdef CYW20719B1
#include <wiced_bt_event.h>
#endif
#ifdef CYW20706A2
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#include <wiced_trans_spi.h>
#endif
#endif
#if ( defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(WICEDX) || defined(CYW20819A1) )
#include <wiced_sleep.h>
#endif
#ifdef CYW20706A2
#include <wiced_power_save.h>
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_SPI)
#include <wiced_hal_pspi.h>
#endif
#endif
#include "hci_control.h"
#include "hci_control_audio.h"
#include "hci_control_rc_controller.h"
#include "hci_control_rc_target.h"
#include "hci_control_test.h"
#include "hci_control_le.h"
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#include <wiced_bt_hfp_ag.h>
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#include <wiced_bt_hfp_hf_int.h>
#endif
#ifdef AMA_ENABLED
#include "hci_control_le_ama.h"
#endif
#include "wiced_app.h"
#include "wiced_app_cfg.h"
#include "a2dp_sink.h"
#ifdef AUTO_ELNA_SWITCH
#include "cycfg_pins.h"
#include "wiced_hal_rfm.h"
#ifndef TX_PU
#define TX_PU   CTX
#endif
#ifndef RX_PU
#define RX_PU   CRX
#endif
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/
#define KEY_INFO_POOL_BUFFER_COUNT  5  //Correspond's to the number of peer devices

#ifdef CYW20819A1
#define WICED_TRANSPORT_BUFFER_COUNT    1
#else
#define WICED_TRANSPORT_BUFFER_COUNT    2
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct hci_control_nvram_chunk
{
    struct hci_control_nvram_chunk *p_next;
    uint16_t nvram_id;
    hci_control_nvram_data_t data;
} hci_control_nvram_chunk_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
#ifdef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
uint8_t avrcp_profile_role = AVRCP_CONTROLLER_ROLE;
uint8_t a2dp_profile_role = A2DP_SINK_ROLE;
#else
uint8_t avrcp_profile_role = AVRCP_TARGET_ROLE;
uint8_t a2dp_profile_role = A2DP_SOURCE_ROLE;
#endif

#if defined(WICED_APP_HFP_AG_INCLUDED)
uint8_t hfp_profile_role = HFP_AUDIO_GATEWAY_ROLE;
#elif defined(WICED_APP_HFP_HF_INCLUDED)
uint8_t hfp_profile_role = HFP_HANDSFREE_UNIT_ROLE;
#endif

hci_control_nvram_chunk_t *p_nvram_first = NULL;
hci_control_nvram_data_t _nvram_data_storage;

wiced_transport_buffer_pool_t* transport_pool;   // Trans pool for sending the RFCOMM data to host
#if BTSTACK_VER >= 0x03000001
wiced_bt_pool_t                *p_key_info_pool;  //Pool for storing the  key info
#else
wiced_bt_buffer_pool_t* p_key_info_pool;  //Pool for storing the  key info
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
static void hci_control_transport_status( wiced_transport_type_t type );
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
#if BTSTACK_VER >= 0x03000001
static void hci_control_transport_tx_cplt_cback(void);
#else
static void hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool);
#endif
static void hci_control_handle_reset_cmd( void );
static void hci_control_handle_trace_enable( uint8_t *p_data );
static void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void hci_control_handle_set_local_bda( uint8_t *p_bda );
static void hci_control_inquiry( uint8_t enable );
static void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void hci_control_handle_set_pairability ( uint8_t pairing_allowed );
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
static void hci_control_handle_enable_disable_coex ( wiced_bool_t enable );
#endif
static void hci_control_handle_read_local_bda( void );
static void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
static void hci_control_handle_read_buffer_stats( void );
static void hci_control_send_device_started_evt( void );
static hci_control_nvram_chunk_t *hci_control_get_nvram_chunk_by_addr(
        wiced_bt_device_address_t bd_addr);
static void hci_control_connection_status_callback (wiced_bt_device_address_t bd_addr,
        uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle,
        wiced_bt_transport_t transport, uint8_t reason);
extern wiced_result_t wiced_bt_avrc_ct_cleanup( void );
extern uint8_t find_index_by_conn_id(uint16_t conn_id);
/******************************************************************************
 *                                Variable/Structure/type Definitions
 ******************************************************************************/

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#if BTSTACK_VER >= 0x03000001
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = WICED_TRANSPORT_BUFFER_COUNT
    },
#endif
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = hci_control_transport_tx_cplt_cback
};

static wiced_bt_local_identity_keys_t local_identity_keys;

/******************************************************
 *               Function Definitions
 ******************************************************/

void hci_control_init(void)
{
    memset(&hci_control_cb, 0, sizeof(hci_control_cb));

    wiced_transport_init(&transport_cfg);
}

void hci_control_post_init(void)
{
#ifdef WICED_APP_LE_INCLUDED
#ifdef AMA_ENABLED
    hci_control_le_ama_enable(&wiced_bt_cfg_settings);
#else
    hci_control_le_enable(&wiced_bt_cfg_settings);
#endif
#endif

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    if (a2dp_profile_role == A2DP_SOURCE_ROLE)
    {
        av_app_init();
    }
#endif
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
    if (a2dp_profile_role == A2DP_SINK_ROLE)
    {
        a2dp_sink_app_init();
    }
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    /* Initialize AVRC Target and Controler */
    if (avrcp_profile_role == AVRCP_CONTROLLER_ROLE)
    {
        hci_control_rc_controller_init();
    }
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    if (avrcp_profile_role == AVRCP_TARGET_ROLE)
    {
        hci_control_rc_target_init();
        wiced_bt_avrc_tg_register();
    }
#endif
#if (defined(WICED_APP_HFP_AG_INCLUDED) || defined(WICED_APP_HFP_HF_INCLUDED))
    /* Perform the rfcomm init before hfp start up */
    if( (wiced_bt_rfcomm_result_t)wiced_bt_rfcomm_init( 200, 5 ) != WICED_BT_RFCOMM_SUCCESS )
    {
        WICED_BT_TRACE("Error Initializing RFCOMM - HFP failed\n");
        return;
    }
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
    hci_control_ag_init();
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
    hci_control_hf_init();
#endif

    // Register connection status change callback
    wiced_bt_dev_register_connection_status_change(hci_control_connection_status_callback);

    // Disable while streaming audio over the uart.
    wiced_bt_dev_register_hci_trace(hci_control_hci_packet_cback);

    // Creating a buffer pool for holding the peer devices's key info
#if BTSTACK_VER >= 0x03000001
    p_key_info_pool = wiced_bt_create_pool("pki", sizeof(hci_control_nvram_chunk_t),
            KEY_INFO_POOL_BUFFER_COUNT, NULL);
#else
    p_key_info_pool = wiced_bt_create_pool(sizeof(hci_control_nvram_chunk_t), KEY_INFO_POOL_BUFFER_COUNT);
#endif
    if (p_key_info_pool == NULL)
        WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");

#ifdef AUTO_ELNA_SWITCH
    wiced_hal_rfm_auto_elna_enable(1, RX_PU);
#endif
#ifdef AUTO_EPA_SWITCH
    wiced_hal_rfm_auto_epa_enable(1, TX_PU);
#endif
}

/*
 *  Process all HCI packet from
 */
#define COD_MAJOR_COMPUTER  1
#define COD_MAJOR_PHONE     2

void hci_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
#if BTSTACK_VER >= 0x03000001
    wiced_transport_send_hci_trace( type, p_data, length  );
#else
#ifdef HCI_TRACE_INCOMING_ACL_DATA_DISABLE
    if (type != HCI_TRACE_INCOMING_ACL_DATA)
#endif
    {
        wiced_transport_send_hci_trace( NULL, type, length, p_data  );
    }
#endif
#endif

    if ( !test_command.test_executing )
        return;

    // If executing test command, need to send Command Complete event back to host app
    if( ( type == HCI_TRACE_EVENT ) && ( length >= 6 ) )
    {
        hci_control_handle_hci_test_event( p_data, length );
    }
}

#ifdef WICED_APP_AMS_INCLUDED
/*
 * Process HCI commands from the MCU
 */
static void hci_control_ams_handle_command(uint8_t index, uint16_t opcode, uint8_t *p_data, uint16_t payload_len)
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t ams_remote_command_id;

    WICED_BT_TRACE("ams:index:%d, opcode:0x%02x\n", index, opcode);

    switch (opcode)
    {
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PLAY;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PAUSE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_NEXT_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_VOLUME_UP;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_VOLUME_DOWN;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_SKIP_FORWARD;
        break;
    case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:
        ams_remote_command_id = AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD;
        break;

    default:
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }

    if (status == HCI_CONTROL_STATUS_SUCCESS)
    {
        wiced_bt_ams_client_send_remote_command(index, ams_remote_command_id);
    }

    hci_control_send_command_status_evt(HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS, status);
}
#endif /* WICED_APP_AMS_INCLUDED */

#ifdef WICED_APP_ANCS_INCLUDED
/*
 * Process HCI commands from the MCU
 */
static void hci_control_ancs_handle_command(uint8_t index, uint16_t opcode, uint8_t *p_data, uint16_t payload_len)
{
    wiced_bool_t result;
    WICED_BT_TRACE("ancs:index:%d\n", index);
    result = wiced_ancs_client_send_remote_command(index, p_data[0] + (p_data[1] << 8) + (p_data[2] << 16) + (p_data[3] << 24), p_data[4]);

    hci_control_send_command_status_evt(HCI_CONTROL_ANCS_EVENT_COMMAND_STATUS,
                                        result ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED );
}
#endif /* WICED_APP_ANCS_INCLUDED */

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;
    uint32_t status;
    uint16_t conn_id;
    uint8_t  index = 0xff;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
#ifndef BTSTACK_VER
        wiced_transport_free_buffer( p_buffer );
#endif
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

#ifdef AMA_ENABLED
    if (!ama_hci_control_command_handler(&status, opcode, p_data, payload_len))
    {
#ifndef BTSTACK_VER
        wiced_transport_free_buffer(p_buffer);
#endif
        return status;
    }
#endif

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

#ifdef WICED_APP_LE_INCLUDED
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    case HCI_CONTROL_GROUP_AUDIO:
        hci_control_audio_handle_command( opcode, p_data, payload_len );
        break;
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    case HCI_CONTROL_GROUP_AVRC_TARGET:
        hci_control_avrc_handle_command( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
        if ((avrcp_profile_role == AVRCP_TARGET_ROLE) &&
            (hci_control_rc_target_is_connected()))
        {
            hci_control_avrc_handle_command( opcode, p_data, payload_len );
        }
        else
#endif
        {
#ifdef WICED_APP_AMS_INCLUDED
            conn_id = (p_data[0]) | (p_data[1] << 8);
            index = find_index_by_conn_id(conn_id);
            if ((index != 0xff) && (wiced_bt_ams_client_connection_check(index)))
            {
                hci_control_ams_handle_command(index, opcode, p_data, payload_len);
            }
            else
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
            {
                hci_control_avrc_handle_ctrlr_command(opcode, p_data, payload_len);
            }
#endif
        }
        break;

#ifdef WICED_APP_TEST_INCLUDED
    case HCI_CONTROL_GROUP_TEST:
        hci_control_test_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_ANCS_INCLUDED
    case HCI_CONTROL_GROUP_ANCS:
        conn_id = (p_data[0]) | (p_data[1] << 8);
        index = find_index_by_conn_id(conn_id);
        if (index != 0xff)
        hci_control_ancs_handle_command( index, opcode, p_data+2, payload_len );
        break;
#endif

#ifdef WICED_APP_HFP_AG_INCLUDED
    case HCI_CONTROL_GROUP_AG:
        hci_control_ag_handle_command( opcode, p_data, payload_len );
        break;
#endif

#ifdef WICED_APP_HFP_HF_INCLUDED
    case HCI_CONTROL_GROUP_HF:
        hci_control_hf_handle_command ( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }
    if (buffer_processed)
    {
#ifndef BTSTACK_VER
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer( p_buffer );
#endif
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], WICED_TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE );
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;
    case HCI_CONTROL_COMMAND_READ_LOCAL_BDA:
        hci_control_handle_read_local_bda();
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation( p_data, p_data[6] );
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats ();
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }

// In SPI transport case, PUART is recommended for debug traces and is set to PUART by default.
#if (WICED_HCI_TRANSPORT != WICED_HCI_TRANSPORT_SPI)
    wiced_set_debug_uart( route_debug );
#endif

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats( void )
{
    uint8_t buff_pools = 0;
#ifdef WICEDX
#define BUFF_POOLS 5
    wiced_bt_buffer_statistics_t buff_stats[BUFF_POOLS];
    buff_pools = BUFF_POOLS;
#else
    wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
    buff_pools = wiced_bt_get_number_of_buffer_pools();
#endif
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for( i=0; i < buff_pools; i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    else {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
    }
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete \n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1 = NULL;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if (wiced_bt_get_buffer_count(p_key_info_pool) <= 0)
            {
                WICED_BT_TRACE( "Err: No more memory for Pairing\n" );
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Handle Get Local BDA command received over UART
 */
void hci_control_handle_read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

    wiced_transport_send_data( HCI_CONTROL_EVENT_READ_LOCAL_BDA, (uint8_t*)bda , 6 );
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing )
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply( accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS : WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            wiced_bt_cfg_settings.l2cap_application.max_links,
            wiced_bt_cfg_settings.l2cap_application.max_channels,
            wiced_bt_cfg_settings.l2cap_application.max_psm,
            wiced_bt_cfg_settings.rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code )
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE( "[hci_control_send_device_error_evt]  app_error_code=0x%02x  fw_error_code=0x%02x\n", event_data[0], event_data[1] );

    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2 );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    int i;

    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status );

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value )
{
    uint8_t buf[10];
    uint8_t *p = &buf[6];
    memcpy( buf, bda, BD_ADDR_LEN );
    *p++ = numeric_value & 0xff;
    *p++ = (numeric_value >> 8) & 0xff;
    *p++ = (numeric_value >> 16) & 0xff;
    *p++ = (numeric_value >> 24) & 0xff;
    wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, 10 );
}


/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    int i;
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
        event_data[i] = wiced_bt_avrc_tg_is_peer_absolute_volume_capable( );
#endif
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_EVENT_DISCONNECTED, event_data, 4 );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x started:%d", __FUNCTION__, handle, started );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_EVENT_STARTED : HCI_CONTROL_AUDIO_EVENT_STOPPED, event_data, 2);
}

/*
 *  send AVRC event to UART
 */
wiced_result_t hci_control_send_avrc_target_event( int type, uint16_t handle )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( type, event_data, 2 );
}

/*********************************************************************************************
 * AVRCP controller event handlers
 *********************************************************************************************/
/*
 *  send avrcp controller complete event to UART
 */
wiced_result_t hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i = 0;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t)];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = status;

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++] = ( handle >> 8 ) & 0xff;

    }
    else
    {
        event_data[i++] = status;
    }

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED, event_data, i );
}

/*
 *  send avrcp controller disconnect complete event to UART
 */
wiced_result_t hci_control_avrc_send_disconnect_complete( uint16_t handle )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] handle: %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, event_data, 2 );
}

/*
 *  send avrcp controller connect complete event to UART
 */
wiced_result_t hci_control_avrc_send_play_status_change( uint16_t handle, uint8_t play_status )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    event_data[2] = play_status & 0xff;                      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send AVRC event to UART
 */

wiced_result_t hci_control_send_avrc_event( int type, uint8_t *p_data, uint16_t data_size )
{
    return wiced_transport_send_data( type, p_data, data_size );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to WICED_FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;
    const hci_control_nvram_data_t *nvram_data = p_data;
    hci_control_nvram_data_t *old_nvram_data = &_nvram_data_storage;
    wiced_bool_t old_nvram_data_exist = WICED_FALSE;
    int bytes_read;

    if (data_len < sizeof(nvram_data->link_keys))
    {
        WICED_BT_TRACE("ERROR %s Invalid data_len %u\n", __func__, data_len);
        return 0;
    }

    /* Read old data in NVRAM */
    bytes_read = hci_control_read_nvram(nvram_id, old_nvram_data, sizeof(*old_nvram_data));
    if (bytes_read == sizeof(*old_nvram_data))
    {
        /* exist */
        old_nvram_data_exist = WICED_TRUE;
    }

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        WICED_BT_TRACE("Failed to alloc NVRAM chunk\n");
        return ( 0 );
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;

    p_nvram_first = p1;

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    WICED_BT_TRACE("[%s] %B", __FUNCTION__, nvram_data->link_keys.bd_addr);

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;

    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys );
    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    if (!from_host)
    {
        p1->data.link_keys = nvram_data->link_keys;
        p1->data.local_identity_keys = local_identity_keys;
        memcpy(p1->data.local_rpa, wiced_btm_get_private_bda(), sizeof(p1->data.local_rpa));
        /* Handle the data item which will be kept original */
        if (old_nvram_data_exist)
        {
            p1->data.audio.a2dp_role = old_nvram_data->audio.a2dp_role;
            p1->data.audio.a2dp_source_audio_route = old_nvram_data->audio.a2dp_source_audio_route;
        }
        else
        {
            p1->data.audio.a2dp_role = A2DP_UNKNOWN_ROLE;
            p1->data.audio.a2dp_source_audio_route = AUDIO_ROUTE_UART;
        }
        WICED_BT_TRACE(" a2dp_role:%d audio_route:%d\n", p1->data.audio.a2dp_role,
                p1->data.audio.a2dp_source_audio_route);
        wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA,
                (void *)&p1->nvram_id, sizeof(p1->nvram_id) + sizeof(p1->data));
    }
    else
    {
        if (data_len >= sizeof(*nvram_data))
        {
            p1->data = *nvram_data;
        }
        WICED_BT_TRACE(" a2dp_role:%d audio_route:%d\n", p1->data.audio.a2dp_role,
                p1->data.audio.a2dp_source_audio_route);

        if (memcmp(&local_identity_keys, &p1->data.local_identity_keys, sizeof(local_identity_keys)) != 0)
        {
            local_identity_keys = p1->data.local_identity_keys;
            wiced_bt_ble_set_local_identity_key_data(p1->data.local_identity_keys.local_key_data);
            wiced_bt_ble_set_resolvable_private_address(p1->data.local_rpa);
            WICED_BT_TRACE("Updated local RPA:%B\n", p1->data.local_rpa);
        }

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
    return sizeof(p1->data);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next)
    {
        WICED_BT_TRACE("find %B %B len:%d\n", p1->data.link_keys.bd_addr, p_data, len);
        if (memcmp(p1->data.link_keys.bd_addr, p_data, len) == 0)
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1, *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if (from_host && wiced_bt_dev_delete_bonded_device(p1->data.link_keys.bd_addr) == WICED_ERROR)
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;

        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if (from_host && wiced_bt_dev_delete_bonded_device(p2->data.link_keys.bd_addr) == WICED_ERROR)
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = MIN(data_len, sizeof(p1->data));
            memcpy(p_data, &p1->data, data_read);
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( void )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        allocated_key_pool_count = 1;

        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            /* If the key buffer pool is becoming full, we need to notify the mcu and disable Pairing.
             * The mcu will need to delete some nvram entries and enable pairing in order to
             * pair with more devices */
            allocated_key_pool_count++;
            if ( ( allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT ) && ( hci_control_cb.pairing_allowed ) )
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data( HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, NULL, 0 );

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
            }

            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
}

/*
 * Get the NVRAM chunk by bd_addr
 */
hci_control_nvram_chunk_t *hci_control_get_nvram_chunk_by_addr(wiced_bt_device_address_t bd_addr)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next)
    {
        if (memcmp(p1->data.link_keys.bd_addr, bd_addr, BD_ADDR_LEN) == 0)
        {
            return p1;
        }
    }

    return NULL;
}

void hci_control_local_identity_keys_update(const wiced_bt_local_identity_keys_t *key)
{
    hci_control_nvram_chunk_t *chunk;

    WICED_BT_TRACE("%s\n", __func__);
    local_identity_keys = *key;

    for (chunk = p_nvram_first; chunk != NULL; chunk = chunk->p_next)
    {
        WICED_BT_TRACE("%s Update %B\n", __func__, chunk->data.link_keys.bd_addr);
        chunk->data.local_identity_keys = local_identity_keys;
        wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA,
                (void *)&chunk->nvram_id, sizeof(chunk->nvram_id) + sizeof(chunk->data));
    }
}

void hci_control_a2dp_role_update(wiced_bt_device_address_t bd_addr, uint8_t a2dp_role)
{
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
    hci_control_nvram_chunk_t *p = hci_control_get_nvram_chunk_by_addr(bd_addr);
    hci_control_nvram_data_t *nvram_data = &_nvram_data_storage;

    if (p == NULL)
    {
        return;
    }

    /* update a2dp_role in chunk */
    p->data.audio.a2dp_role = a2dp_role;

    /* write nvram */
    memcpy(nvram_data, &p->data, sizeof(*nvram_data));
    hci_control_write_nvram(p->nvram_id, sizeof(*nvram_data), nvram_data, WICED_FALSE);
#endif /* (defined(WICED_APP_AUDIO_SRC_INCLUDED) && (defined(WICED_APP_AUDIO_SNK_INCLUDED) */
}

uint8_t hci_control_a2dp_role_get(wiced_bt_device_address_t bd_addr)
{
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
    hci_control_nvram_chunk_t *p = hci_control_get_nvram_chunk_by_addr(bd_addr);

    if (p == NULL)
    {
        return A2DP_UNKNOWN_ROLE;
    }

    return p->data.audio.a2dp_role;
#else
    return A2DP_UNKNOWN_ROLE;
#endif /* defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED)  */
}

void hci_control_a2dp_source_audio_route_update(wiced_bt_device_address_t bd_addr,
        uint8_t audio_route)
{
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
    hci_control_nvram_chunk_t *p = hci_control_get_nvram_chunk_by_addr(bd_addr);
    hci_control_nvram_data_t *nvram_data = &_nvram_data_storage;

    if (p == NULL)
    {
        return;
    }

    /* update a2dp_role in chunk */
    p->data.audio.a2dp_source_audio_route = audio_route;

    /* write nvram */
    memcpy(nvram_data, &p->data, sizeof(*nvram_data));
    hci_control_write_nvram(p->nvram_id, sizeof(*nvram_data), nvram_data, WICED_FALSE);
#endif /* defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) */
}

uint8_t hci_control_a2dp_source_audio_route_get(wiced_bt_device_address_t bd_addr)
{
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
    hci_control_nvram_chunk_t *p = hci_control_get_nvram_chunk_by_addr(bd_addr);

    if (p == NULL)
    {
        /* default is AUDIO_ROUTE_UART */
        return AUDIO_ROUTE_UART;
    }

    return p->data.audio.a2dp_source_audio_route;
#else
    return AUDIO_ROUTE_UART;
#endif /* defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) */
}

/*
 * Remote Control can work a target or a controller.  This function sets up the appropriate role.
 */
void hci_control_switch_avrcp_role(uint8_t new_role)
{
#if ( defined(WICED_APP_AUDIO_RC_TG_INCLUDED) && defined(WICED_APP_AUDIO_RC_CT_INCLUDED) )
    WICED_BT_TRACE("[%s] Switch from %d to %d\n", __FUNCTION__, avrcp_profile_role, new_role);

    if (new_role != avrcp_profile_role)
    {
        switch (avrcp_profile_role)
        {
        case AVRCP_TARGET_ROLE:
            /* Shutdown the avrcp target */
            wiced_bt_avrc_tg_initiate_close();

            /* Initialize the avrcp controller */
            hci_control_rc_controller_init();

            avrcp_profile_role = new_role;
            break;

        case AVRCP_CONTROLLER_ROLE:
            /* Shutdown the avrcp controller */
            wiced_bt_avrc_ct_cleanup();

            /* Initialize the avrcp target */
            hci_control_rc_target_init();
            wiced_bt_avrc_tg_register();

            avrcp_profile_role = new_role;
            break;

        default:
            break;
        }
    }
#endif
}

#ifdef WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK
/*
 * A2DP can work a sink or a source.  This function sets up the appropriate role.
 */
void hci_control_switch_a2dp_role(uint8_t new_role)
{
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
    wiced_result_t result;

    WICED_BT_TRACE("[%s] Switch from %d to %d\n", __FUNCTION__, a2dp_profile_role, new_role);
    if (new_role == a2dp_profile_role)
    {
        WICED_BT_TRACE(">> Role unchange\n");
        return;
    }

    /* close original role */
    if (a2dp_profile_role == A2DP_SOURCE_ROLE)
    {
        result = av_app_deinit();
    }
    else if (a2dp_profile_role == A2DP_SINK_ROLE)
    {
        result = a2dp_sink_app_deinit();
    }
    else
    {
        result = WICED_SUCCESS;
        WICED_BT_TRACE(">> Unknown orignal a2dp role\n");
    }

    if (result != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Error: fail to close original role\n");
        return;
    }

    /* switch to new role */
    a2dp_profile_role = new_role;
    if (a2dp_profile_role == A2DP_SOURCE_ROLE)
    {
        av_app_init();
    }
    else if (a2dp_profile_role == A2DP_SINK_ROLE)
    {
        a2dp_sink_app_init();
    }
    else
    {
        WICED_BT_TRACE(">> Switch a2dp to idle\n");
    }
#endif /* defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) */

    return;
}
#endif /* WICED_APP_AUDIO_ROLE_SERVICE_SWITCH_WITH_SNK */

/*
 * hci_control_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
#if BTSTACK_VER >= 0x03000001
static void hci_control_transport_tx_cplt_cback(void)
{
	return;
}
#else
static void hci_control_transport_tx_cplt_cback( wiced_transport_buffer_pool_t* p_pool )
{
    WICED_BT_TRACE( " hci_control_transport_tx_cplt_cback %x \n", p_pool );
}
#endif

static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    hci_control_send_device_started_evt();
}

/*
 * Connection status callback
 */
void hci_control_connection_status_callback (wiced_bt_device_address_t bd_addr,
        uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle,
        wiced_bt_transport_t transport, uint8_t reason)
{
    wiced_result_t result = WICED_ERROR;

    WICED_BT_TRACE("%s %B is_connected:%d reason:0x%x result:%d handle: %d, transport:%d\n",
            __FUNCTION__, bd_addr, is_connected, reason, result, handle, transport);

    /* Update information. */
    switch (transport)
    {
    case BT_TRANSPORT_BR_EDR:
#if ( defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) )
        if (is_connected)
        {
            /* check for a2dp_role */
            uint8_t a2dp_role = hci_control_a2dp_role_get(bd_addr);
            if (a2dp_role == A2DP_SINK_ROLE)
            {
                WICED_BT_TRACE("Switch to A2DP Sink Role\n");
                hci_control_switch_a2dp_role(A2DP_SINK_ROLE);
                hci_control_switch_avrcp_role(AVRCP_CONTROLLER_ROLE);
            }
            else if (a2dp_role == A2DP_SOURCE_ROLE)
            {
                WICED_BT_TRACE("Switch to A2DP Source Role\n");
                hci_control_switch_a2dp_role(A2DP_SOURCE_ROLE);
                hci_control_switch_avrcp_role(AVRCP_TARGET_ROLE);
            }
            else
            {
                WICED_BT_TRACE("Keep default A2DP role\n");
            }
        }
#endif /* defined(WICED_APP_AUDIO_SRC_INCLUDED) && defined(WICED_APP_AUDIO_SNK_INCLUDED) */
        break;
    case BT_TRANSPORT_LE:
        // Nothing to do
        break;
    default:
        return;
    }
}

void hci_control_switch_hfp_role( uint8_t new_role )
{
#if (defined(WICED_APP_HFP_AG_INCLUDED) && defined(WICED_APP_HFP_HF_INCLUDED))
    WICED_BT_TRACE("[%s] Switch from %d to %d\n", __FUNCTION__, hfp_profile_role, new_role);
    /* switch to new role */
    hfp_profile_role = new_role;
#endif /* (defined(WICED_APP_HFP_AG_INCLUDED) && defined(WICED_APP_HFP_HF_INCLUDED)) */
    return;
}
