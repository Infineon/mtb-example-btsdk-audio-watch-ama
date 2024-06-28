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
 * This file implements the entry point of the Wiced Application
 */
#ifdef AMA_ENABLED
#include <ama.h>
#endif
#include <sparcommon.h>
#ifdef WICED_APP_AMS_INCLUDED
#include <wiced_bt_ams.h>
#endif
#ifdef WICED_APP_HFP_AG_INCLUDED
#include "hci_control_hfp_ag.h"
#include <wiced_bt_hfp_ag.h>
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
#include "hci_control_hfp_hf.h"
#include "wiced_bt_hfp_hf_int.h"
#endif
#include <wiced_bt_stack.h>
#include <wiced_bt_trace.h>
#include <wiced_memory.h>
#if (defined(CYW20721B2) || defined(CYW43012C0))
#include <wiced_utilities.h>
#endif
#include <wiced_platform.h>
#include <wiced_hal_gpio.h>
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
#include "a2dp_sink.h"
#endif
#include "hci_control.h"
#include "hci_control_le.h"
#include "hci_control_rc_controller.h"
#include "le_peripheral.h"
#include "wiced_app_cfg.h"
#include "wiced_app.h"
#ifdef CYW9BT_AUDIO
#include <wiced_audio_manager.h>
#endif

typedef struct
{
    wiced_bt_device_address_t   identity_addr;
    wiced_bt_device_address_t   random_addr;
} app_identity_random_mapping_t;

#define ADDR_MAPPING_MAX_COUNT 15 //as same as APP_CFG_ULP_MAX_CONNECTION
static app_identity_random_mapping_t addr_mapping[ADDR_MAPPING_MAX_COUNT] = {0};

#define WICED_HS_EIR_BUF_MAX_SIZE 264


static void write_eir(void);
static wiced_result_t btm_event_handler(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_result_t btm_enabled_event_handler(wiced_bt_dev_enabled_t *event_data);
static void ama_event_forward_at_command_handler(const char *at_command);
static void ama_event_media_control_handler(wiced_bt_ama_media_control_t control);
static void ama_audio_record_pre_start(wiced_bool_t is_hfp_active);
static void ama_audio_record_post_stop(wiced_bool_t is_hfp_active);
extern uint8_t find_index_by_conn_id(uint16_t conn_id);

/*
 * Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
    wiced_result_t result;

    wiced_hal_puart_init();
    wiced_hal_puart_configuration(3000000, PARITY_NONE, STOP_BIT_2);
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    hci_control_init();

    result = wiced_bt_stack_init(btm_event_handler, &wiced_bt_cfg_settings, wiced_app_cfg_buf_pools);
    if (WICED_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR bt_stack_init %u\n", result);
        return;
    }

    result = wiced_audio_buffer_initialize(wiced_bt_audio_buf_config);
    if (WICED_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR audio_buffer_init %u\n", result);
        return;
    }

#ifdef WICED_APP_LE_INCLUDED
    hci_control_le_init();
#endif

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    le_peripheral_app_init();
#endif

#ifdef AMA_ENABLED
    ama_init();
#endif

    WICED_BT_TRACE("WatchAma App Start\n");
}

void write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p, *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    //Allocating a buffer from the public pool
    pBuf = (uint8_t *)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );

    WICED_BT_TRACE("%s %x\n", __func__, pBuf);

    if ( !pBuf )
    {
        return;
    }

    p = pBuf;

    length = strlen( (char *)wiced_bt_cfg_settings.device_name );
    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy( p, wiced_bt_cfg_settings.device_name, length );
    p += length;

    // Add other BR/EDR UUIDs
    p_tmp = p;      // We don't now the number of UUIDs for the moment
    p++;
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
#ifdef WICED_APP_AUDIO_SRC_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SOURCE);       nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_TG_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REM_CTRL_TARGET); nb_uuid++;
#endif
#ifdef WICED_APP_AUDIO_RC_CT_INCLUDED
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AV_REMOTE_CONTROL);  nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);         nb_uuid++;
#endif
#if (defined(WICED_APP_HFP_AG_INCLUDED) || defined(WICED_APP_HFP_HF_INCLUDED))
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HEADSET);            nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_HF_HANDSFREE);       nb_uuid++;
    UINT16_TO_STREAM(p, UUID_SERVCLASS_GENERIC_AUDIO);      nb_uuid++;
#endif

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf ), MIN( p - ( uint8_t* )pBuf, 100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer( pBuf );

    return;
}

app_identity_random_mapping_t * get_addr_mapping_by_identity_addr(wiced_bt_device_address_t identity_addr)
{
    for (int i=0; i<ADDR_MAPPING_MAX_COUNT;i++)
    {
        if (memcmp(identity_addr, addr_mapping[i].identity_addr, sizeof(wiced_bt_device_address_t))==0)
            return &addr_mapping[i];
    }
    return NULL;
}

app_identity_random_mapping_t * get_empty_addr_mapping()
{
    wiced_bt_device_address_t empty_addr={0};

    return get_addr_mapping_by_identity_addr(empty_addr);
}

app_identity_random_mapping_t * get_addr_mapping_by_random_addr(wiced_bt_device_address_t random_addr)
{
    for (int i=0; i<ADDR_MAPPING_MAX_COUNT;i++)
    {
        if (memcmp(random_addr, addr_mapping[i].random_addr, sizeof(wiced_bt_device_address_t))==0)
            return &addr_mapping[i];
    }
    return NULL;
}

void app_paired_device_link_keys_update( wiced_bt_management_evt_data_t *p_event_data )
{
    int nvram_id, bytes_written;
    wiced_bt_device_address_t zero_bda = {0};
    wiced_bt_management_evt_data_t modified_event_data;

    /* Check if we already have information saved for this bd_addr */
    if ( memcmp(p_event_data->paired_device_link_keys_update.key_data.static_addr, zero_bda, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        // Save link key infomation for static_addr
        memcpy(&modified_event_data, p_event_data, sizeof(wiced_bt_management_evt_data_t));
        memcpy(modified_event_data.paired_device_link_keys_update.bd_addr, modified_event_data.paired_device_link_keys_update.key_data.static_addr, sizeof(wiced_bt_device_address_t));

        if ( ( nvram_id = hci_control_find_nvram_id( modified_event_data.paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id( );
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &modified_event_data.paired_device_link_keys_update, WICED_FALSE );

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, modified_event_data.paired_device_link_keys_update.bd_addr);
    }
    else
    {
        if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id( );
            WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
        }
        bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
    }

    if ( memcmp(p_event_data->paired_device_link_keys_update.key_data.static_addr, zero_bda, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        app_identity_random_mapping_t * addr_map = get_empty_addr_mapping();
        if (addr_map != NULL)
        {
            memcpy(addr_map->random_addr, p_event_data->paired_device_link_keys_update.bd_addr, sizeof(wiced_bt_device_address_t));
            memcpy(addr_map->identity_addr, p_event_data->paired_device_link_keys_update.key_data.static_addr, sizeof(wiced_bt_device_address_t));
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT identity addr [%B] random addr [%B]\n", addr_map->identity_addr, addr_map->random_addr);
        }
    }
}

wiced_result_t btm_event_handler(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl;
    uint8_t pairing_result;
    wiced_bt_device_link_keys_t *link_keys;
    int nvram_id;
    int bytes_written;
    int bytes_read;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;

    WICED_BT_TRACE("btm_event_handler %u\n", event);

    switch (event)
    {
        /* Bluetooth stack enabled */
        case BTM_ENABLED_EVT:
            result = btm_enabled_event_handler(&p_event_data->enabled);
            break;

        case BTM_DISABLED_EVT:
            hci_control_send_device_error_evt(p_event_data->disabled.reason, 0);
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            if (p_event_data->user_confirmation_request.just_works)
            {
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                hci_control_send_user_confirmation_request_evt(p_event_data->user_confirmation_request.bd_addr, p_event_data->user_confirmation_request.numeric_value );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT (%B, io_cap: 0x%02X)\n",
                    p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                    p_event_data->pairing_io_capabilities_br_edr_response.io_cap);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for LE */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
                hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
                app_identity_random_mapping_t * addr_map = get_addr_mapping_by_random_addr(p_event_data->pairing_complete.bd_addr);
                if ( addr_map != NULL )
                {
                    hci_control_send_pairing_completed_evt( pairing_result, addr_map->identity_addr );
                }
            }
            WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE("Encryption Status:(%B) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result);

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
            if (p_encryption_status->transport == BT_TRANSPORT_LE)
                le_peripheral_encryption_status_changed(p_encryption_status);
#endif
            hci_control_send_encryption_changed_evt(p_encryption_status->result, p_encryption_status->bd_addr);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed );
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            link_keys = &p_event_data->paired_device_link_keys_update;
            app_paired_device_link_keys_update(p_event_data);
            if (link_keys->key_data.br_edr_key_type != HCI_LKEY_TYPE_UNKNOWN)
            {
                const uint8_t *key = link_keys->key_data.br_edr_key;
                WICED_BT_TRACE("Link Key:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x type:%u\n",
                        key[0], key[1], key[2], key[3], key[4], key[5], key[6], key[7],
                        key[8], key[9], key[10], key[11], key[12], key[13], key[14], key[15],
                        link_keys->key_data.br_edr_key_type);
            }

            if (link_keys->key_data.le_keys_available_mask & (BTM_LE_KEY_PENC | BTM_LE_KEY_LENC))
            {
                WICED_BT_TRACE("Security level:0x%02x\n", link_keys->key_data.le_keys.sec_level);
            }
            if (link_keys->key_data.le_keys_available_mask & BTM_LE_KEY_PID)
            {
                const uint8_t *irk = link_keys->key_data.le_keys.irk;
                WICED_BT_TRACE("Peer IRK:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        irk[0], irk[1], irk[2], irk[3], irk[4], irk[5], irk[6], irk[7],
                        irk[8], irk[9], irk[10], irk[11], irk[12], irk[13], irk[14], irk[15]);
                WICED_BT_TRACE("Peer addr:%Btype:%u\n", link_keys->key_data.static_addr, link_keys->key_data.static_addr_type);
            }

            if (link_keys->key_data.le_keys_available_mask & BTM_LE_KEY_LENC)
            {
                const uint8_t *ltk = link_keys->key_data.le_keys.lltk;
                WICED_BT_TRACE("Local LTK:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                        ltk[0], ltk[1], ltk[2], ltk[3], ltk[4], ltk[5], ltk[6], ltk[7],
                        ltk[8], ltk[9], ltk[10], ltk[11], ltk[12], ltk[13], ltk[14], ltk[15]);
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            hci_control_local_identity_keys_update(&p_event_data->local_identity_keys_update);
            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            result = WICED_BT_NO_RESOURCES;
            break;

#ifdef WICED_APP_LE_INCLUDED
        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            hci_control_le_scan_state_changed(p_event_data->ble_scan_state_changed);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            WICED_BT_TRACE("BLE_ADVERT_STATE_CHANGED_EVT:%d\n", p_event_data->ble_advert_state_changed);
            hci_control_le_advert_state_changed(p_event_data->ble_advert_state_changed);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("BTM LE Connection Update event status %d BDA [%B] interval %d latency %d supervision timeout %d\n",
                    p_event_data->ble_connection_param_update.status,
                    p_event_data->ble_connection_param_update.bd_addr,
                    p_event_data->ble_connection_param_update.conn_interval,
                    p_event_data->ble_connection_param_update.conn_latency,
                    p_event_data->ble_connection_param_update.supervision_timeout);
            break;
#endif

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE("Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr,
                    p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
#ifdef WICED_APP_HFP_AG_INCLUDED
            if (hfp_profile_role == HFP_AUDIO_GATEWAY_ROLE)
            {
                hfp_ag_sco_management_callback( event, p_event_data );
            }
#endif
#ifdef WICED_APP_HFP_HF_INCLUDED
            if (hfp_profile_role == HFP_HANDSFREE_UNIT_ROLE)
            {
                hci_control_hf_sco_management_callback( event, p_event_data );
            }
#endif
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

wiced_result_t btm_enabled_event_handler(wiced_bt_dev_enabled_t *event_data)
{
    const ama_config_t ama_config = {
        .settings = &wiced_bt_cfg_settings,
        .event_forward_at_command_handler = ama_event_forward_at_command_handler,
        .event_media_control_handler = ama_event_media_control_handler,
        .audio_record_pre_start_handler = ama_audio_record_pre_start,
        .audio_record_post_stop_handler = ama_audio_record_post_stop,
    };
    wiced_result_t result;

    write_eir();

    /* create SDP records */
    if (!wiced_bt_sdp_db_init((uint8_t *)wiced_app_cfg_sdp_record, wiced_app_cfg_sdp_record_get_size()))
    {
        WICED_BT_TRACE("ERROR sdp_db_init\n");
        return WICED_BT_ERROR;
    }

#ifdef AMA_ENABLED
    result = ama_post_init_hci_based(&ama_config);
    if (WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("ERROR ama_post_init %u\n", result);
        return WICED_BT_ERROR;
    }
#endif

    hci_control_post_init();

#ifdef CYW9BT_AUDIO
    /* Initialize AudioManager */
    wiced_am_init();
#endif

    WICED_BT_TRACE("Free Bytes After Init:%d\n", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}

void ama_event_forward_at_command_handler(const char *at_command)
{
    WICED_BT_TRACE("Ama forward AT command %s\n", at_command);
}

void ama_event_media_control_handler(wiced_bt_ama_media_control_t control)
{
    uint8_t ams_index = 0;
    uint16_t ama_conn_id = 0;
    const uint8_t ams_command_ids[] = {
        [WICED_BT_AMA_MEDIA_CONTROL_PLAY] = AMS_REMOTE_COMMAND_ID_PLAY,
        [WICED_BT_AMA_MEDIA_CONTROL_PAUSE] = AMS_REMOTE_COMMAND_ID_PAUSE,
        [WICED_BT_AMA_MEDIA_CONTROL_NEXT] = AMS_REMOTE_COMMAND_ID_NEXT_TRACK,
        [WICED_BT_AMA_MEDIA_CONTROL_PREVIOUS] = AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK,
    };
    const uint8_t avrc_ids[] = {
        [WICED_BT_AMA_MEDIA_CONTROL_PLAY] = AVRC_ID_PLAY,
        [WICED_BT_AMA_MEDIA_CONTROL_PAUSE] = AVRC_ID_PAUSE,
        [WICED_BT_AMA_MEDIA_CONTROL_NEXT] = AVRC_ID_FORWARD,
        [WICED_BT_AMA_MEDIA_CONTROL_PREVIOUS] = AVRC_ID_BACKWARD,
    };

    WICED_BT_TRACE("Ama media %u\n", control);

    if (control >= _countof(ams_command_ids))
    {
        WICED_BT_TRACE("ERROR Ama media %u\n", control);
        return;
    }
#ifdef AMA_ENABLED
    ama_conn_id = ama_get_conn_id();
    ams_index = find_index_by_conn_id(ama_conn_id);
#endif
    if (wiced_bt_ams_client_connection_check(ams_index))
    {
        wiced_bt_ams_client_send_remote_command(ams_index, ams_command_ids[control]);
    }
    else
    {
        const wiced_result_t result = hci_control_rc_controller_send_pass_through_cmd(avrc_ids[control], AVRC_STATE_PRESS);
        if (WICED_BT_SUCCESS != result)
        {
            WICED_BT_TRACE("ERROR send_pass_through_cmd %u\n", result);
        }
    }
}

void ama_audio_record_pre_start(wiced_bool_t is_hfp_active)
{
    if (!is_hfp_active)
    {
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
        a2dp_sink_am_stream_suspend();
#endif
    }
}

void ama_audio_record_post_stop(wiced_bool_t is_hfp_active)
{
    if (!is_hfp_active)
    {
#ifdef WICED_APP_AUDIO_SNK_INCLUDED
        a2dp_sink_am_stream_resume();
#endif
    }
}
