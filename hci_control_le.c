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

#ifdef WICED_APP_LE_INCLUDED

/** @file
 *
 * This file implement BTLE application controlled over UART.
 * The GATT database is defined in this file and is not changed by the MCU.
 *
 */
#include <wiced_bt_dev.h>
#include <wiced_bt_l2c.h>
#include <wiced_memory.h>
#include <wiced_transport.h>
#include <hci_control_api.h>
#include "GeneratedSource/cycfg_gatt_db.h"
#include "hci_control.h"
#include "hci_control_le.h"
#include "le_peripheral.h"
#include "wiced_app_cfg.h"
#if BTSTACK_VER >= 0x03000001
#include "cycfg_gap.h"
#endif

/******************************************************
 *                     Constants
 ******************************************************/

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
hci_control_le_cb_t le_control_cb;

wiced_timer_t hci_control_le_connect_timer;

BD_ADDR  hci_control_le_remote_bdaddr;

hci_control_le_pending_tx_buffer_t hci_control_le_pending_tx_buffer;

#if BTSTACK_VER >= 0x03000001
wiced_bt_db_hash_t watch_db_hash;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );
#if BTSTACK_VER >= 0x03000001
static wiced_bt_gatt_status_t         hci_control_le_write_handler( uint16_t conn_idx, wiced_bt_gatt_write_req_t * p_data );
#else
static wiced_bt_gatt_status_t         hci_control_le_write_handler( uint16_t conn_idx, wiced_bt_gatt_write_t * p_data );
#endif
static void                   hci_control_le_notification_handler( uint16_t conn_idx, uint16_t handle, uint8_t *p_data, uint16_t len );
static void                   hci_control_le_indication_handler( uint16_t conn_idx, uint16_t handle, uint8_t *p_data, uint16_t len );
static void                   hci_control_le_process_data( uint16_t type, uint16_t conn_idx, uint16_t handle, uint8_t *data, int len );
static void                   hci_control_le_send_scan_state_event( uint8_t state );
static void                   hci_control_le_send_advertisement_state_event( uint8_t state );
static void                   hci_control_le_send_advertisement_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static void                   hci_control_le_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
static void                   hci_control_le_send_disconnect_evt( uint8_t reason, uint16_t conn_idx );
static void                   hci_control_le_send_discover_complete( uint16_t conn_idx );
static void                   hci_control_le_send_discovered_service16( uint16_t conn_idx, uint16_t uuid, uint16_t s_handle, uint16_t e_handle );
static void                   hci_control_le_send_discovered_service128( uint16_t conn_idx, uint8_t *uuid, uint16_t s_handle, uint16_t e_handle );
static void                   hci_control_le_send_discovered_characteristic16( uint16_t conn_idx, uint16_t char_handle, uint8_t properties, uint16_t handle, uint16_t uuid );
static void                   hci_control_le_send_discovered_characteristic128( uint16_t conn_idx, uint16_t char_handle, uint8_t properties, uint16_t handle, uint8_t *uuid );
static void                   hci_control_le_send_discovered_descriptor16( uint16_t conn_idx, uint16_t handle, uint16_t uuid );
static void                   hci_control_le_send_discovered_descriptor128( uint16_t conn_idx, uint16_t handle, uint8_t *uuid );
static void                   hci_control_le_send_read_rsp( uint16_t conn_idx, uint8_t *data, int len );
static void                   hci_control_le_send_read_req( uint16_t conn_idx, uint16_t handle );
static void                   hci_control_le_send_write_completed( uint16_t conn_idx, uint8_t result );
static wiced_bt_gatt_status_t   hci_control_le_gatt_congestion_callback( wiced_bt_gatt_congestion_event_t *p_congestion);
static void                   hci_control_le_handle_delete_nvram_info_and_initiate_pair ( BD_ADDR addr, uint16_t nvram_id );
static void                   hci_control_le_handle_get_identity_address ( BD_ADDR bd_addr );
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
static void                   hci_control_le_handle_set_channel_classification ( uint8_t ble_channel_map[ BTM_BLE_CHNL_MAP_SIZE ] );
#endif
static void                   hci_control_le_set_advertisement_data(const wiced_bt_cfg_settings_t *settings);

extern uint32_t wiced_bt_ble_get_available_tx_buffers(void);

/*
 * Initialize LE Control
 */
void hci_control_le_init( void )
{
    memset( &le_control_cb, 0, sizeof( le_control_cb ) );
    memset( &hci_control_le_pending_tx_buffer, 0, sizeof( hci_control_le_pending_tx_buffer ) );
}

/*
 * Enable LE Control
 */
void hci_control_le_enable(const wiced_bt_cfg_settings_t *settings)
{
    wiced_bt_gatt_status_t     gatt_status;

    WICED_BT_TRACE( "hci_control_le_enable\n" );

    /* GATT registration */
    gatt_status = wiced_bt_gatt_register( hci_control_le_gatt_callback );

    WICED_BT_TRACE( "wiced_bt_gatt_register status %d\n", gatt_status );

    /*  GATT DB Initialization */
#if BTSTACK_VER >= 0x03000001
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len,
            watch_db_hash);
#else
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
#endif

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    hci_control_le_set_advertisement_data(settings);

    /* Initialize connection timer */
    wiced_init_timer( &hci_control_le_connect_timer, &hci_control_le_connect_timeout, 0, WICED_SECONDS_TIMER );
}

/*
 * hci_control_le_set_advertisement_data
 */
static void hci_control_le_set_advertisement_data(const wiced_bt_cfg_settings_t *settings)
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
#else
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = app_gap_device_name_len;
    adv_elem[num_elem].p_data       = ( uint8_t* )app_gap_device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
#endif
}
/*
 * Process advertisement packet received
 */
void hci_control_le_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    if ( p_scan_result )
    {
        WICED_BT_TRACE( " Device : %B\n", p_scan_result->remote_bd_addr );
        hci_control_le_send_advertisement_report( p_scan_result, p_adv_data );
    }
    else
    {
        WICED_BT_TRACE( " Scan completed\n" );
    }
}

#if BTSTACK_VER >= 0x03000001
/*
 * Helper function for mapping between conn_id and conn_idx
 */
uint16_t hci_control_le_allocate_conn_cb (void)
{
    uint8_t i;
    for (i = 0; i < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)); i++)
        if (le_control_cb.conn[i].conn_id == 0)
            return i;

    WICED_BT_TRACE("hci_control_le_allocate_conn_cb fail!!! \n");
    return 0xffff;
}

uint16_t hci_control_le_get_conn_idx (uint16_t conn_id)
{
    uint8_t i;
    for (i = 0; i < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)); i++)
        if (le_control_cb.conn[i].conn_id == conn_id)
            return i;

    WICED_BT_TRACE("hci_control_le_get_conn_idx fail %d !!! \n", conn_id);
    return 0xffff;
}

uint16_t hci_control_le_get_conn_id (uint16_t conn_idx)
{
    if (conn_idx < (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)))
        return le_control_cb.conn[conn_idx].conn_id;

    WICED_BT_TRACE("hci_control_le_get_conn_id fail %d !!! \n", conn_idx);
    return 0xffff;
}
#endif /* BTSTACK_VER */

uint16_t hci_control_le_get_conn_idx_from_event(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    uint16_t conn_idx = 0xFFFF;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->connection_status.conn_id);
#else
        conn_idx = p_data->connection_status.conn_id;
#endif
        break;

    case GATT_OPERATION_CPLT_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->operation_complete.conn_id);
#else
        conn_idx = p_data->operation_complete.conn_id;
#endif
        break;

    case GATT_DISCOVERY_RESULT_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->discovery_result.conn_id);
#else
        conn_idx = p_data->discovery_result.conn_id;
#endif
        break;

    case GATT_DISCOVERY_CPLT_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->discovery_complete.conn_id);
#else
        conn_idx = p_data->discovery_complete.conn_id;
#endif
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->attribute_request.conn_id);
#else
        conn_idx = p_data->attribute_request.conn_id;
#endif
        break;

    case GATT_CONGESTION_EVT:
#if BTSTACK_VER >= 0x03000001
        conn_idx = hci_control_le_get_conn_idx(p_data->congestion.conn_id);
#else
        conn_idx = p_data->congestion.conn_id;
#endif
        break;

    default:
        break;
    }

    if (conn_idx == 0xFFFF || conn_idx >= (sizeof(le_control_cb.conn)/sizeof(hci_control_le_conn_state_t)))
    {
        WICED_BT_TRACE("%s fail to get conn_idx for evt %d\n", __FUNCTION__, event);
        conn_idx = 0;
    }

    return conn_idx;
}

/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
#if BTSTACK_VER >= 0x03000001
    uint32_t       conn_idx = hci_control_le_allocate_conn_cb();
#else
    uint32_t       conn_idx = p_status->conn_id;
#endif
    uint8_t        role;
    wiced_bt_ble_sec_action_type_t  encryption_type;

    wiced_bt_dev_get_role( p_status->bd_addr, &role, p_status->transport);
    le_control_cb.conn[conn_idx].role = role;

    WICED_BT_TRACE("%s id %u conn_idx %d bd (%B) role %u\n", __func__, p_status->conn_id, conn_idx, p_status->bd_addr, role);

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    memcpy( le_control_cb.conn[conn_idx].bd_addr, p_status->bd_addr, BD_ADDR_LEN );
    le_control_cb.conn[conn_idx].state      = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_idx].conn_id    = p_status->conn_id;
#if BTSTACK_VER >= 0x03000001
    le_control_cb.conn[conn_idx].peer_mtu   = GATT_BLE_DEFAULT_MTU_SIZE;
#else
    le_control_cb.conn[conn_idx].peer_mtu   = GATT_DEF_BLE_MTU_SIZE;
#endif

    // if we connected as a central configure peripheral to enable notifications
    if ( hci_control_cb.pairing_allowed && ( role == HCI_ROLE_CENTRAL ) )
    {
        wiced_result_t res;
        if ( ( hci_control_find_nvram_id( p_status->bd_addr, BD_ADDR_LEN ) ) == 0 )
        {
            res = wiced_bt_dev_sec_bond( p_status->bd_addr, p_status->addr_type,
                    p_status->transport, 0, NULL );
            WICED_BT_TRACE( "Central Role: after bond res:%d\n", res );
        }
        else // device already paired previous. Just do encryption
        {
            encryption_type = BTM_BLE_SEC_ENCRYPT;
            res = wiced_bt_dev_set_encryption(p_status->bd_addr, p_status->transport, &encryption_type);
            WICED_BT_TRACE( "Central Role: after encrypt res:%d\n",res );
        }
    }
    else
    {
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
#ifndef AMA_ENABLED
        // ask central to set preferred connection parameters
        // For better audio quality higher connection interval is preferable
        wiced_bt_l2cap_update_ble_conn_params( p_status->bd_addr, 112, 128, 0, 200 );
#endif
        le_peripheral_connection_up(p_status);
#endif
    }

    hci_control_le_send_connect_event(0 /* TBD should come from p_status */,
            p_status->bd_addr, p_status->conn_id, role);
    return ( WICED_SUCCESS );
}

/*
* Process connection down event
*/
wiced_result_t hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t          conn_idx = hci_control_le_get_conn_idx(p_status->conn_id);
#else
    uint16_t          conn_idx = p_status->conn_id;
#endif

    WICED_BT_TRACE( "le_connection_down conn_idx:%x Disc_Reason: %02x\n", conn_idx, p_status->reason );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    if ( conn_idx < (LE_CONTROL_MAX_CONNECTIONS + 1) )
    {
        le_control_cb.conn[conn_idx].state   = LE_CONTROL_STATE_IDLE;
        le_control_cb.conn[conn_idx].conn_id = 0;

        if ( ( conn_idx == hci_control_le_pending_tx_buffer.tx_buf_conn_idx ) &&
                ( hci_control_le_pending_tx_buffer.tx_buf_saved ) )
        {
            hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
        }

#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
        if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
        {
            le_peripheral_connection_down(p_status);
        }
#endif

        hci_control_le_send_disconnect_evt( p_status->reason, conn_idx );
    }

    return ( WICED_SUCCESS );
}


/*
* Process connection status callback
*/
wiced_result_t hci_control_le_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return hci_control_le_connection_up( p_status );
    }
    else
    {
        return hci_control_le_connection_down( p_status );
    }
}

/*
 * Operation complete received from the GATT server
 */
wiced_result_t hci_control_le_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(p_complete->conn_id);
#else
    uint16_t conn_idx = p_complete->conn_id;
#endif

    switch ( p_complete->op )
    {
    case GATTC_OPTYPE_DISCOVERY:
        WICED_BT_TRACE( "!!! Disc compl conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        break;

#if BTSTACK_VER >= 0x03000001
    case GATTC_OPTYPE_READ_HANDLE:
#else
    case GATTC_OPTYPE_READ:
#endif
        // read response received, pass it up and set state to idle
        WICED_BT_TRACE( "Read response conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        if ( le_control_cb.conn[conn_idx].state == LE_CONTROL_STATE_READ_VALUE )
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
            if ( p_complete->status != WICED_SUCCESS )
                hci_control_le_send_read_rsp( conn_idx, NULL, 0 );
            else
                hci_control_le_send_read_rsp( conn_idx, p_complete->response_data.att_value.p_data,
                        p_complete->response_data.att_value.len );
        }
        break;

#if BTSTACK_VER >= 0x03000001
    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
    case GATTC_OPTYPE_EXECUTE_WRITE:
#else
    case GATTC_OPTYPE_WRITE:
    case GATTC_OPTYPE_EXE_WRITE:
#endif
        // write response received, pass it up and set state to idle
        WICED_BT_TRACE( "Write response conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        if ( le_control_cb.conn[conn_idx].state == LE_CONTROL_STATE_WRITE_VALUE )
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
            hci_control_le_send_write_completed( conn_idx, p_complete->status );
        }
        break;

#if BTSTACK_VER >= 0x03000001
    case GATTC_OPTYPE_CONFIG_MTU:
#else
    case GATTC_OPTYPE_CONFIG:
#endif
        WICED_BT_TRACE( "Config conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        WICED_BT_TRACE( "Notification conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        hci_control_le_notification_handler( conn_idx,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE( "Indication conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );
        hci_control_le_indication_handler( conn_idx,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * Discovery result received from the GATT server
 */
wiced_result_t hci_control_le_gatt_disc_result_cb( wiced_bt_gatt_discovery_result_t *p_result )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(p_result->conn_id);
#else
    uint16_t conn_idx = p_result->conn_id;
#endif

    WICED_BT_TRACE( "Discovery result conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );

    switch ( le_control_cb.conn[conn_idx].state )
    {
    case LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES:
        if ( ( p_result->discovery_type == GATT_DISCOVER_SERVICES_ALL ) ||
            ( p_result->discovery_type == GATT_DISCOVER_SERVICES_BY_UUID ) )
        {
            WICED_BT_TRACE( "Service s:%04x e:%04x uuid:%04x\n", p_result->discovery_data.group_value.s_handle,
                    p_result->discovery_data.group_value.e_handle, p_result->discovery_data.group_value.service_type.uu.uuid16 );

            // services on the server can be based on 2 or 16 bytes UUIDs
            if ( p_result->discovery_data.group_value.service_type.len == 2 )
            {
                hci_control_le_send_discovered_service16( conn_idx, p_result->discovery_data.group_value.service_type.uu.uuid16,
                        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle );
            }
            else
            {
                hci_control_le_send_discovered_service128( conn_idx, p_result->discovery_data.group_value.service_type.uu.uuid128,
                        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle );
            }
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTICS )
        {
            WICED_BT_TRACE( "Found cha - uuid:%04x, hdl:%04x\n", p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16, p_result->discovery_data.characteristic_declaration.handle );

            // characteristic can be based on 2 or 16 bytes UUIDs
            if ( p_result->discovery_data.characteristic_declaration.char_uuid.len == 2 )
            {
                hci_control_le_send_discovered_characteristic16( conn_idx, p_result->discovery_data.characteristic_declaration.handle,
                        p_result->discovery_data.characteristic_declaration.characteristic_properties,
                        p_result->discovery_data.characteristic_declaration.val_handle,
                        p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16 );
            }
            else
            {
                hci_control_le_send_discovered_characteristic128( conn_idx, p_result->discovery_data.characteristic_declaration.handle,
                        p_result->discovery_data.characteristic_declaration.characteristic_properties,
                        p_result->discovery_data.characteristic_declaration.val_handle,
                        p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid128 );
            }
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_DESCRIPTORS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
        {
            WICED_BT_TRACE( "Found descr - uuid:%04x handle:%04x\n", p_result->discovery_data.char_descr_info.type.uu.uuid16, p_result->discovery_data.char_descr_info.handle );

            // descriptor can be based on 2 or 16 bytes UUIDs
            if ( p_result->discovery_data.char_descr_info.type.len == 2 )
            {
                hci_control_le_send_discovered_descriptor16( conn_idx, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid16 );
            }
            else
            {
                hci_control_le_send_discovered_descriptor128( conn_idx, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid128 );
            }
        }
        break;

    default:
        WICED_BT_TRACE( "ignored\n" );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * process discovery complete notification from the stack
 */
wiced_result_t hci_control_le_gatt_disc_comp_cb( wiced_bt_gatt_discovery_complete_t *p_data )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(p_data->conn_id);
#else
    uint16_t conn_idx = p_data->conn_id;
#endif
    // if we got here peer returned no more services, or we read up to the handle asked by client, report complete
    le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
    hci_control_le_send_discover_complete(conn_idx);

    return ( WICED_SUCCESS );
}

/* Get a Value */
#if BTSTACK_VER >= 0x03000001
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t attr_handle, uint16_t conn_idx,
        wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_data, uint16_t len_requested)
#else
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t attr_handle, uint16_t conn_idx, uint8_t *p_val, uint16_t len_requested, uint16_t *p_len )
#endif
{
    int                    i;
    wiced_bool_t           is_handle_in_table = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#endif

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            is_handle_in_table = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= len_requested)
            {
                // Value fits within the supplied buffer; copy over the value
#if BTSTACK_VER >= 0x03000001
                int attr_len_to_copy = app_gatt_db_ext_attr_tbl[i].cur_len;
                if (attr_len_to_copy != 0)
                {
                    uint8_t *from;
                    int     to_copy = MIN(len_requested, attr_len_to_copy - p_read_data->offset);
                    from = ((uint8_t *)app_gatt_db_ext_attr_tbl[i].p_data) + p_read_data->offset;
                    if (to_copy < 0)
                    {
                        res = WICED_BT_GATT_INVALID_OFFSET;
                    }
                    else
                    {
                        wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_copy, from, NULL);
                        res = WICED_BT_GATT_SUCCESS;
                    }
                }
#else /* !BTSTACK_VER */
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;
#endif /* BTSTACK_VER */
            }
            else
            {
                // Value to read will not fit within the buffer
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!is_handle_in_table)
    {
        // TBD. If handle is not contained within external lookup table pass the read req to the host
        // res = WICED_BT_GATT_PENDING;
    }

#if BTSTACK_VER >= 0x03000001
    if (res != WICED_BT_GATT_SUCCESS)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                res);
    }
#endif
    return res;
}

/* Set a Value */
wiced_bt_gatt_status_t hci_control_le_set_value( uint16_t attr_handle, uint16_t conn_idx, uint8_t *p_val, uint16_t len )
{
    int                    i;
    wiced_bool_t           is_handle_in_table = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            is_handle_in_table = WICED_TRUE;
            // Verify that size constraints have been met

            if ((app_gatt_db_ext_attr_tbl[i].max_len >= len))
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                // Value to write does not meet size constraints
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!is_handle_in_table)
    {
        res = WICED_BT_GATT_PENDING;
    }

    return res;
}


/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */
#if BTSTACK_VER >= 0x03000001
wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_idx, wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_t *p_req, uint16_t len_requested)
{
    return hci_control_le_get_value(p_req->handle, conn_idx, opcode, p_req, len_requested);
}
#else
wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_idx, wiced_bt_gatt_read_t *p_req )
{
    return hci_control_le_get_value(p_req->handle, conn_idx, p_req->p_val, *p_req->p_val_len, p_req->p_val_len);
}
#endif
/*
 * The function invoked on timeout of hci_control_le_connect_timer
 */
void hci_control_le_connect_timeout( uint32_t count )
{
    WICED_BT_TRACE( "%s\n", __FUNCTION__ );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    /* Cancel connection request */
    wiced_bt_gatt_cancel_connect( hci_control_le_remote_bdaddr, WICED_TRUE );
}

/*
 * This function is called when peer issues a Write request to access characteristics values
 * in the GATT database
 */
#if BTSTACK_VER >= 0x03000001
wiced_bt_gatt_status_t hci_control_le_write_handler( uint16_t conn_idx, wiced_bt_gatt_write_req_t * p_req )
#else
wiced_bt_gatt_status_t hci_control_le_write_handler( uint16_t conn_idx, wiced_bt_gatt_write_t *p_req )
#endif
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    WICED_BT_TRACE( "hci_control_le_write_handler: conn_idx:%d handle %04x\n", conn_idx, p_req->handle );
    switch ( p_req->handle )
    {
    default:
        status = hci_control_le_set_value(p_req->handle, conn_idx, p_req->p_val, p_req->val_len);

        if (status == WICED_BT_GATT_PENDING)
            hci_control_le_process_data( HCI_CONTROL_GATT_EVENT_WRITE_REQUEST, conn_idx, p_req->handle, p_req->p_val, p_req->val_len );

        break;
    }
    return ( status );
}

wiced_result_t hci_control_le_write_exec_handler( uint16_t conn_idx, wiced_bt_gatt_exec_flag_t flag )
{
    return ( WICED_SUCCESS );
}

wiced_result_t hci_control_le_mtu_handler( uint16_t conn_idx, uint16_t mtu )
{
    le_control_cb.conn[conn_idx].peer_mtu   = mtu;

#if BTSTACK_VER >= 0x03000001
    wiced_bt_gatt_server_send_mtu_rsp(hci_control_le_get_conn_id(conn_idx),
            mtu, wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
#endif

    return ( WICED_SUCCESS );
}

/*
 * Process indication confirm.
 */
wiced_result_t  hci_control_le_conf_handler( uint16_t conn_idx, uint16_t handle )
{
    WICED_BT_TRACE( "hci_control_le_conf_handler conn_idx:%d state:%d\n", conn_idx, le_control_cb.conn[conn_idx].state );

    if ( le_control_cb.conn[conn_idx].state == LE_CONTROL_STATE_INDICATE_VALUE )
    {
        le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
        hci_control_le_send_write_completed( conn_idx, 0 );
    }
    return WICED_SUCCESS;
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;

#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(p_req->conn_id);

    WICED_BT_TRACE( "GATT request conn_idx:%d type:%d\n", conn_idx, p_req->opcode );

    switch ( p_req->opcode )
    {
        case GATT_REQ_READ:
            result = hci_control_le_read_handler( conn_idx, p_req->opcode, &p_req->data.read_req, p_req->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            /* TODO: support for read_by_type handler */
            WICED_BT_TRACE("READ_BY_TYPE not support\n");
            wiced_bt_gatt_server_send_error_rsp(p_req->conn_id, p_req->opcode,
                    p_req->data.read_by_type.s_handle, WICED_BT_GATT_REQ_NOT_SUPPORTED);
            break;

        case GATT_REQ_WRITE:
            result = hci_control_le_write_handler( conn_idx, &p_req->data.write_req );
            if (result == WICED_BT_GATT_SUCCESS)
            {
                wiced_bt_gatt_server_send_write_rsp(p_req->conn_id, p_req->opcode, p_req->data.write_req.handle);
            }
            else
            {
                wiced_bt_gatt_server_send_error_rsp(p_req->conn_id, p_req->opcode, p_req->data.write_req.handle,
                        result);
            }
             break;

        case GATT_REQ_EXECUTE_WRITE:
            result = hci_control_le_write_exec_handler( conn_idx, p_req->data.exec_write_req.exec_write );
            wiced_bt_gatt_server_send_execute_write_rsp(p_req->conn_id, p_req->opcode);
            break;

        case GATT_REQ_MTU:
            result = hci_control_le_mtu_handler( conn_idx, p_req->data.remote_mtu );
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = hci_control_le_conf_handler( conn_idx, p_req->data.confirm.handle );
            break;

       default:
            WICED_BT_TRACE("Unhandled GATT request\n");
            break;
    }

#else /* !BTSTACK_VER */
    uint16_t conn_idx = p_req->conn_id;

    WICED_BT_TRACE( "GATT request conn_idx:%d type:%d\n", conn_idx, p_req->request_type );

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hci_control_le_read_handler( conn_idx, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hci_control_le_write_handler( conn_idx, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = hci_control_le_write_exec_handler( conn_idx, p_req->data.exec_write );
            break;

        case GATTS_REQ_TYPE_MTU:
            result = hci_control_le_mtu_handler( conn_idx, p_req->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler( conn_idx, p_req->data.handle );
            break;

       default:
            break;
    }

#endif /* BTSTACK_VER */

    return result;
}

wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;
    uint16_t conn_idx;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback( &p_data->connection_status );
        break;

    case GATT_OPERATION_CPLT_EVT:
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
        conn_idx = hci_control_le_get_conn_idx_from_event(event, p_data);
        if (le_control_cb.conn[conn_idx].role == HCI_ROLE_PERIPHERAL)
            result = le_peripheral_gatt_operation_complete(&p_data->operation_complete);
        else
#endif
            result = hci_control_le_gatt_operation_comp_cb( &p_data->operation_complete );
        break;

    case GATT_DISCOVERY_RESULT_EVT:
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
        conn_idx = hci_control_le_get_conn_idx_from_event(event, p_data);
        if (le_control_cb.conn[conn_idx].role == HCI_ROLE_PERIPHERAL)
            result = le_peripheral_gatt_discovery_result( &p_data->discovery_result );
        else
#endif
            result = hci_control_le_gatt_disc_result_cb( &p_data->discovery_result );
        break;

    case GATT_DISCOVERY_CPLT_EVT:
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
        conn_idx = hci_control_le_get_conn_idx_from_event(event, p_data);
        if (le_control_cb.conn[conn_idx].role == HCI_ROLE_PERIPHERAL)
            result = le_peripheral_gatt_discovery_complete( &p_data->discovery_complete );
        else
#endif
            result = hci_control_le_gatt_disc_comp_cb( &p_data->discovery_complete );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb( &p_data->attribute_request );
        break;

    case GATT_CONGESTION_EVT:
        result = hci_control_le_gatt_congestion_callback( &p_data->congestion );
        break;

#if BTSTACK_VER >= 0x03000001
    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = wiced_bt_get_buffer(p_data->buffer_request.len_requested);
        p_data->buffer_request.buffer.p_app_ctxt = wiced_bt_free_buffer;
        result = WICED_BT_GATT_SUCCESS;
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            void (*pfn_free)(uint8_t *) =
                (void (*)(uint8_t *))p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_data->buffer_xmitted.p_app_data);

            result = WICED_BT_GATT_SUCCESS;
        }
        break;
#endif /* BTSTACK_VER */

    default:
        break;
    }

    return result;
}

/*
 * This function sends write to the peer GATT server
 * */
#if BTSTACK_VER >= 0x03000001
wiced_bt_gatt_status_t hci_control_le_send_write( uint8_t conn_idx, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_opcode_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    wiced_bt_gatt_write_hdr_t hdr;
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);

    // Allocating a buffer to send the write request
    uint8_t *p_write = (uint8_t *)wiced_bt_get_buffer(len);

    if ( p_write )
    {
        hdr.handle   = attr_handle;
        hdr.offset   = 0;
        hdr.len      = len;
        hdr.auth_req = GATT_AUTH_REQ_NONE;
        memcpy(p_write, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write ( conn_id, type, &hdr, p_write, NULL);

        WICED_BT_TRACE( "wiced_bt_gatt_client_send_write status:%d\n", status );

        wiced_bt_free_buffer( p_write );
    }

    return ( status );
}
#else /* !BTSTACK_VER */
wiced_bt_gatt_status_t hci_control_le_send_write( uint8_t conn_idx, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_write_type_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;
    uint16_t conn_id = conn_idx;

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( GATT_RESPONSE_SIZE( len ) );

    if ( p_write )
    {
        p_write->handle   = attr_handle;
        p_write->offset   = 0;
        p_write->len      = len;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        memcpy( p_write->value, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( conn_id, type, p_write );

        WICED_BT_TRACE( "wiced_bt_gatt_send_write status:%d", status );

        wiced_bt_free_buffer( p_write );
    }
    return ( status );
}
#endif /* BTSTACK_VER */

/*
 * Num complete callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_congestion_callback( wiced_bt_gatt_congestion_event_t *p_congestion )
{
    uint16_t             handle;
    uint16_t             conn_id;
    static int           last_connection_serviced = 0;
    int                  i;
    wiced_bt_gatt_status_t ret_val = WICED_BT_GATT_SUCCESS;

    //WICED_BT_TRACE( "hci_control_le_num_complete_callback available buffs:%d\n", wiced_bt_ble_get_available_tx_buffers( ) );

    if ( hci_control_le_pending_tx_buffer.tx_buf_saved && ( wiced_bt_ble_get_available_tx_buffers( ) > 1 ) )
    {
        WICED_BT_TRACE( "service conn_idx:%d\n", hci_control_le_pending_tx_buffer.tx_buf_conn_idx );

        // saved tx buffer can be write command, or notification.
        if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND )
        {
#if BTSTACK_VER >= 0x03000001
            ret_val = hci_control_le_send_write( hci_control_le_pending_tx_buffer.tx_buf_conn_idx, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_data, hci_control_le_pending_tx_buffer.tx_buf_len, GATT_CMD_WRITE );
#else
            ret_val = hci_control_le_send_write( hci_control_le_pending_tx_buffer.tx_buf_conn_idx, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_data, hci_control_le_pending_tx_buffer.tx_buf_len, GATT_WRITE_NO_RSP );
#endif
            if (WICED_BT_GATT_SUCCESS != ret_val)
            {
                return ret_val;
            }
        }
        else if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_NOTIFY )
        {
#if BTSTACK_VER >= 0x03000001
            conn_id = hci_control_le_get_conn_id(hci_control_le_pending_tx_buffer.tx_buf_conn_idx);
            ret_val = wiced_bt_gatt_server_send_notification( conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_len, hci_control_le_pending_tx_buffer.tx_buf_data,
                            NULL );
#else
            conn_id = hci_control_le_pending_tx_buffer.tx_buf_conn_idx;
            ret_val = wiced_bt_gatt_send_notification( conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_len, hci_control_le_pending_tx_buffer.tx_buf_data );
#endif
            if (WICED_BT_GATT_SUCCESS != ret_val)
            {
                return ret_val;
            }
        }
        else
        {
            WICED_BT_TRACE( "bad packet queued:%d\n", hci_control_le_pending_tx_buffer.tx_buf_type );
        }

        // tx_buffer sent successfully, clear tx_buf_saved flag
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;

        hci_control_le_send_write_completed( hci_control_le_pending_tx_buffer.tx_buf_conn_idx, 0 );
    }

    return WICED_BT_GATT_SUCCESS;
}


/*
 * Stack runs the advertisement state machine switching between high duty, low
 * duty, no advertisements, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_advert_state_changed( wiced_bt_ble_advert_mode_t mode )
{
    uint8_t hci_control_le_event;

    WICED_BT_TRACE( "Advertisement State Change:%d\n", mode );

    switch ( mode )
    {
    case BTM_BLE_ADVERT_OFF:
        hci_control_le_event = LE_ADV_STATE_NO_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_HIGH:
    case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
    case BTM_BLE_ADVERT_NONCONN_HIGH:
    case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
        hci_control_le_event = LE_ADV_STATE_HIGH_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_LOW:
    case BTM_BLE_ADVERT_UNDIRECTED_LOW:
    case BTM_BLE_ADVERT_NONCONN_LOW:
    case BTM_BLE_ADVERT_DISCOVERABLE_LOW:
        hci_control_le_event = LE_ADV_STATE_LOW_DISCOVERABLE;
        break;
    default:
        hci_control_le_event = LE_ADV_STATE_NO_DISCOVERABLE;
		break;
    }
    hci_control_le_send_advertisement_state_event( hci_control_le_event );
}

/*
 * Stack runs the scan state machine switching between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state )
{
    uint8_t hci_control_le_event;

    WICED_BT_TRACE( "Scan State Changed:%d\n", state );

    switch ( state )
    {
    case BTM_BLE_SCAN_TYPE_NONE:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_HIGH_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_LOW_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_LOW_SCAN;
        break;
    default:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;
        break;
    }
    hci_control_le_send_scan_state_event( hci_control_le_event );
}

/*
 * This function is called when notification is received from the connected GATT Server
 */
void hci_control_le_notification_handler( uint16_t conn_idx, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    WICED_BT_TRACE( "Notification conn_idx:%d handle:%04x len:%d\n", conn_idx, handle, len );

    hci_control_le_process_data( HCI_CONTROL_GATT_EVENT_NOTIFICATION, conn_idx, handle, p_data, len );
}

/*
 * This function is called when indication is received from the connected GATT Server
 */
void hci_control_le_indication_handler( uint16_t conn_idx, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    uint16_t conn_id;
    WICED_BT_TRACE( "Indication conn_idx:%d handle:%04x len:%d\n", conn_idx, handle, len );

    hci_control_le_process_data( HCI_CONTROL_GATT_EVENT_INDICATION, conn_idx, handle, p_data, len );

#if BTSTACK_VER >= 0x03000001
    conn_id = hci_control_le_get_conn_id(conn_idx);
    wiced_bt_gatt_client_send_indication_confirm( conn_id, handle );
#else
    conn_id = conn_idx;
    wiced_bt_gatt_send_indication_confirm( conn_id, handle );
#endif
}

/*
 * Handle various LE GAP and GATT commands received from the Control app.
 */
void hci_control_le_handle_command( uint16_t cmd_opcode, uint8_t* p, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_LE_COMMAND_SCAN:
        hci_control_le_handle_scan_cmd( (wiced_bool_t)p[0], (wiced_bool_t)p[1] );
        break;

    case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        hci_control_le_handle_advertise_cmd( (wiced_bool_t)p[0] );
        break;

    case HCI_CONTROL_LE_COMMAND_CONNECT:
        hci_control_le_handle_connect_cmd( *p, &p[1] );
        break;

    case HCI_CONTROL_LE_COMMAND_CANCEL_CONNECT:
        hci_control_le_handle_cancel_connect_cmd( *p, &p[1] );
        break;

    case HCI_CONTROL_LE_COMMAND_DISCONNECT:
        hci_control_le_handle_disconnect_cmd( p[0] + ( p[1] << 8 ) );
        break;

    case HCI_CONTROL_LE_RE_PAIR:
        hci_control_le_handle_delete_nvram_info_and_initiate_pair ( &p[0], p[6] + ( p[7] << 8 ) );
        break;

    case HCI_CONTROL_LE_COMMAND_GET_IDENTITY_ADDRESS:
        hci_control_le_handle_get_identity_address ( &p[0] );
        break;
#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
    case HCI_CONTROL_LE_COMMAND_SET_CHANNEL_CLASSIFICATION:
        hci_control_le_handle_set_channel_classification( &p[0] );
        break;
#endif
    case HCI_CONTROL_GATT_COMMAND_DISCOVER_SERVICES:
        hci_control_le_handle_service_discovery( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), p[4] + ( p[5] << 8 ) );
        break;

    case HCI_CONTROL_GATT_COMMAND_DISCOVER_CHARACTERISTICS:
        hci_control_le_handle_characteristic_discovery( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), p[4] + ( p[5] << 8 ) );
        break;

    case HCI_CONTROL_GATT_COMMAND_DISCOVER_DESCRIPTORS:
        hci_control_le_handle_descriptor_discovery( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), p[4] + ( p[5] << 8 ) );
        break;

    case HCI_CONTROL_GATT_COMMAND_READ_REQUEST:
        hci_control_le_handle_read_req( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ) );
        break;

    case HCI_CONTROL_GATT_COMMAND_READ_RESPONSE:
        hci_control_le_handle_read_rsp( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), &p[4], data_len - 4 );
        break;

    case HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND:
        hci_control_le_handle_write_cmd( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), &p[4], data_len - 4 );
        break;

    case HCI_CONTROL_GATT_COMMAND_WRITE_REQUEST:
        hci_control_le_handle_write_req( p[0] + ( p[1] << 8 ), p[2] + ( p[3] << 8 ), &p[4], data_len - 4 );
        break;

    case HCI_CONTROL_GATT_COMMAND_NOTIFY:
    case HCI_CONTROL_GATT_COMMAND_INDICATE:
    case HCI_CONTROL_GATT_COMMAND_INDICATE_CONFIRM:
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
        break;

    default:
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_UNKNOWN_COMMAND );
        break;
    }
}

/*
 * handle scan command from UART
 */
void hci_control_le_handle_scan_cmd( wiced_bool_t enable, wiced_bool_t filter_duplicates )
{
    wiced_result_t result;
    uint8_t status;

    if ( enable )
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, filter_duplicates, hci_control_le_scan_result_cback );
    }
    else
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, filter_duplicates, hci_control_le_scan_result_cback );
    }
    WICED_BT_TRACE( "hci_control_le_handle_scan_cmd:%d status:%x\n", enable, result );

    if( ( result == WICED_BT_SUCCESS ) || ( result  == WICED_BT_PENDING) )
    {
        status = HCI_CONTROL_STATUS_SUCCESS;
    }
    else
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }

    WICED_BT_TRACE( "hci_control_le_handle_scan_cmd:%d status:%x\n", enable, status );
    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status );
}

/*
 * handle advertise command from UART
 */
void hci_control_le_handle_advertise_cmd( wiced_bool_t enable )
{
    if ( enable )
    {
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    }
    else
    {
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}


/*
 * handle connect command from UART
 */
void hci_control_le_handle_connect_cmd( uint8_t addr_type, BD_ADDR addr )
{
    wiced_bool_t status;

    /* Initiate the connection */
    STREAM_TO_BDADDR(hci_control_le_remote_bdaddr,addr);

    status = wiced_bt_gatt_le_connect( hci_control_le_remote_bdaddr, addr_type,
            BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE );

    /* Start le connection timer */
    if ( status == WICED_TRUE )
        wiced_start_timer( &hci_control_le_connect_timer, LE_CONTROL_CONNECT_TIMEOUT );

    WICED_BT_TRACE( "wiced_bt_gatt_connect status %d, BDA %B, Addr Type %x\n",
                                          status, hci_control_le_remote_bdaddr, addr_type );

    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
        status == WICED_TRUE ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
}

/*
 * handle cancel connect command from uart
 */
void hci_control_le_handle_cancel_connect_cmd( uint8_t addr_type, BD_ADDR addr )
{
    BD_ADDR                bda;
    STREAM_TO_BDADDR(bda,addr);
    wiced_bt_gatt_cancel_connect ( bda, WICED_TRUE );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  handle disconnect command from UART
 */
void hci_control_le_handle_disconnect_cmd( uint16_t conn_id )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif
    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
#if BTSTACK_VER >= 0x03000001
    else if ( le_control_cb.conn[conn_idx].conn_id == 0 )
#else
    else if ( le_control_cb.conn[conn_idx].conn_id != conn_id )
#endif
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_NOT_CONNECTED );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

        le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_DISCONNECTING;

        wiced_bt_gatt_disconnect( conn_id );
    }
}

/*
 * handle start services discovery command from UART
 */
void hci_control_le_handle_service_discovery( uint16_t conn_id, uint16_t s_handle, uint16_t e_handle )
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( ( s_handle > e_handle ) || ( s_handle == 0 ) || ( e_handle == 0 ) )
    {
        WICED_BT_TRACE( "illegal handles:%04x-%04x\n", s_handle, e_handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
        // perform service search
        memset( &params, 0, sizeof( params ) );
        params.s_handle = s_handle;
        params.e_handle = e_handle;

        WICED_BT_TRACE( "discover services conn_idx:%d %04x-%04x\n", conn_idx, s_handle, e_handle );
#if BTSTACK_VER >= 0x03000001
        if ( ( rc = wiced_bt_gatt_client_send_discover( conn_id, GATT_DISCOVER_SERVICES_ALL, &params ) ) == WICED_BT_GATT_SUCCESS )
#else
        if ( ( rc = wiced_bt_gatt_send_discover( conn_id, GATT_DISCOVER_SERVICES_ALL, &params ) ) == WICED_BT_GATT_SUCCESS )
#endif
        {
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES;
        }
        else
        {
            WICED_BT_TRACE( "discover failed:0x%x\n", rc );
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
        }
    }
}

/*
 * handle start characteristics discovery command from UART
 */
void hci_control_le_handle_characteristic_discovery( uint16_t conn_id, uint16_t s_handle, uint16_t e_handle )
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( ( s_handle > e_handle ) || ( s_handle == 0 ) || ( e_handle == 0 ) )
    {
        WICED_BT_TRACE( "illegal handles:%04x-%04x\n", s_handle, e_handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
        // perform characteristics search
        memset( &params, 0, sizeof( params ) );
        params.s_handle = s_handle;
        params.e_handle = e_handle;

#if BTSTACK_VER >= 0x03000001
        if ( ( rc = wiced_bt_gatt_client_send_discover( conn_id, GATT_DISCOVER_CHARACTERISTICS, &params ) ) == WICED_BT_GATT_SUCCESS )
#else
        if ( ( rc = wiced_bt_gatt_send_discover( conn_id, GATT_DISCOVER_CHARACTERISTICS, &params ) ) == WICED_BT_GATT_SUCCESS )
#endif
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS;
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
        }
        else
        {
            WICED_BT_TRACE( "discover failed:0x%x\n", rc );
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
        }
    }
}

/*
 * handle start descriptors discovery command from UART
 */
void hci_control_le_handle_descriptor_discovery( uint16_t conn_id, uint16_t s_handle, uint16_t e_handle )
{
    wiced_bt_gatt_discovery_param_t params;
    wiced_bt_gatt_status_t          rc;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( ( s_handle > e_handle ) || ( s_handle == 0 ) || ( e_handle == 0 ) )
    {
        WICED_BT_TRACE( "illegal handles:%04x-%04x\n", s_handle, e_handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
        memset( &params, 0, sizeof( params ) );
        params.s_handle = s_handle;
        params.e_handle = e_handle;

#if BTSTACK_VER >= 0x03000001
        if ( ( rc = wiced_bt_gatt_client_send_discover( conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &params ) ) == WICED_BT_GATT_SUCCESS )
#else
        if ( ( rc = wiced_bt_gatt_send_discover( conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &params ) ) == WICED_BT_GATT_SUCCESS )
#endif
        {
            // perform find info procedure to read all descriptors
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_DISCOVER_DESCRIPTORS;
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
        }
        else
        {
            WICED_BT_TRACE( "discover failed:0x%x\n", rc );
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
        }
    }
}

void hci_control_le_handle_read_req( uint16_t conn_id, uint16_t handle )
{
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( handle == 0 )
    {
        WICED_BT_TRACE( "illegal handle:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
        wiced_bt_gatt_status_t status;
#if BTSTACK_VER >= 0x03000001
        status = wiced_bt_gatt_client_send_read_handle( conn_id, handle, 0, NULL, 0, GATT_AUTH_REQ_NONE);

#else /* !BTSTACK_VER */
        wiced_bt_gatt_read_param_t read_req;

        // execute read procedure
        memset( &read_req, 0, sizeof( wiced_bt_gatt_read_param_t ) );

        read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
        read_req.by_handle.handle = handle;

        status = wiced_bt_gatt_send_read( conn_id, GATT_READ_BY_HANDLE, &read_req );
#endif /* BTSTACK_VER */
        if (status == WICED_BT_GATT_SUCCESS)
        {
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_READ_VALUE;
        }
        else
        {
            hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
        }
    }
}

/*
 * Host replied with the data to be passed in the GATT Read Response
 */
void hci_control_le_handle_read_rsp( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    wiced_bt_gatt_status_t status;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
#if BTSTACK_VER >= 0x03000001
        status = wiced_bt_gatt_server_send_read_handle_rsp( conn_id, GATT_RSP_READ, len, p_data, NULL );
#else
        status = wiced_bt_gatt_send_response( WICED_BT_GATT_SUCCESS, conn_id, handle, len, 0, p_data );
#endif

        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status == WICED_BT_GATT_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED );

        WICED_BT_TRACE( "read_rsp conn_idx:%d status:%d\n", conn_idx, status );
    }
}

/*
 * Process write command received over UART.  Return WICED_TRUE if buffer has been
 * queued, WICED_FALSE if buffer has been shipped.
 */
wiced_bool_t hci_control_le_handle_write_cmd( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    wiced_bt_gatt_status_t status;

#if BTSTACK_VER >= 0x03000001
        uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
        uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
#if BTSTACK_VER >= 0x03000001
    else if ( ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE ) )
#else
    else if ( ( le_control_cb.conn[conn_idx].conn_id != conn_idx ) || ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE ) )
#endif
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( handle == 0 )
    {
        WICED_BT_TRACE( "illegal handle:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else if ( hci_control_le_pending_tx_buffer.tx_buf_saved )
    {
        WICED_BT_TRACE( "client sent next packet before previous is acked:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_IN_PROGRESS );
    }
    else
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved   = WICED_TRUE;
        hci_control_le_pending_tx_buffer.tx_buf_conn_idx= conn_idx;
        hci_control_le_pending_tx_buffer.tx_buf_type    = HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND;
        hci_control_le_pending_tx_buffer.tx_buf_handle  = handle;
        hci_control_le_pending_tx_buffer.tx_buf_len     = len <= sizeof( hci_control_le_pending_tx_buffer.tx_buf_data ) ?
                                                                             len : sizeof( hci_control_le_pending_tx_buffer.tx_buf_data );
        memcpy( hci_control_le_pending_tx_buffer.tx_buf_data, p_data, hci_control_le_pending_tx_buffer.tx_buf_len );

        WICED_BT_TRACE( "write_cmd %d avail bufs %d\n", conn_idx, wiced_bt_ble_get_available_tx_buffers( ) );

        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

        // If there are tx buffers send Write Command
        if ( wiced_bt_ble_get_available_tx_buffers( ) > 1 )
        {
#if BTSTACK_VER >= 0x03000001
            status = hci_control_le_send_write( conn_idx, handle, p_data, len, GATT_CMD_WRITE );
#else
            status = hci_control_le_send_write( conn_idx, handle, p_data, len, GATT_WRITE_NO_RSP );
#endif
            if ( status != WICED_BT_GATT_SUCCESS )
            {
                WICED_BT_TRACE( "Failed to send:%x\n", status );
            }
            else
            {
                // Write Command sent successfully, clear tx_buf_saved flag
                hci_control_le_pending_tx_buffer.tx_buf_saved  = WICED_FALSE;

                hci_control_le_send_write_completed( conn_idx, status );
            }
        }
        else
        {
            WICED_BT_TRACE( "queued conn_idx:%04x\n", conn_idx );
        }
    }
    return ( hci_control_le_pending_tx_buffer.tx_buf_saved );
}

void hci_control_le_handle_write_req( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    wiced_bt_gatt_status_t status;
#if BTSTACK_VER >= 0x03000001
    uint8_t *p_write_req;
    wiced_bt_gatt_write_hdr_t hdr;
    uint16_t conn_idx = hci_control_le_get_conn_idx(conn_id);
#else
    uint16_t write_req_len;
    wiced_bt_gatt_value_t *p_write_req;
    uint16_t conn_idx = conn_id;
#endif

    if ( conn_idx > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_idx:%04x\n", conn_idx );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_idx].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_idx].state );
    }
    else if ( handle == 0 )
    {
        WICED_BT_TRACE( "illegal handle:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
#ifdef WICED_APP_LE_PERIPHERAL_CLIENT_INCLUDED
    else if (le_control_cb.conn[conn_idx].role != HCI_ROLE_CENTRAL)
    {
        WICED_BT_TRACE( "GATT Command not allowed for Peripheral Connection\n");
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_DISALLOWED );
    }
#endif
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

        // perform write procedure
#if BTSTACK_VER >= 0x03000001
        p_write_req = (uint8_t *)wiced_bt_get_buffer(len);

        WICED_BT_TRACE( "write_cmd %d %x\n", conn_idx, p_write_req );
        if( p_write_req == NULL )
        {
            return;
        }

        memset( p_write_req, 0, len);

        hdr.handle   = handle;
        hdr.offset   = 0;
        hdr.len      = len;
        hdr.auth_req = GATT_AUTH_REQ_NONE;

        memcpy( p_write_req, p_data, len );

        // Change the state to indicate that we are waiting for write response
        le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_WRITE_VALUE;

        status = wiced_bt_gatt_client_send_write( conn_id, GATT_REQ_WRITE, &hdr, p_write_req, NULL );

        WICED_BT_TRACE( "send write. offset %d len %d status %d\n", 0, len, status );

        wiced_bt_free_buffer( p_write_req );

#else /* BTSTACK_VER */
        write_req_len = ( sizeof( wiced_bt_gatt_value_t ) - 1 + len );
        p_write_req = (wiced_bt_gatt_value_t *)wiced_bt_get_buffer( write_req_len );

        WICED_BT_TRACE( "write_cmd %d %x\n", conn_idx, p_write_req );
        if( p_write_req == NULL )
        {
            return;
        }

        memset( p_write_req, 0, write_req_len );

        p_write_req->handle   = handle;
        p_write_req->offset   = 0;
        p_write_req->len      = len;
        p_write_req->auth_req = GATT_AUTH_REQ_NONE;

        memcpy( p_write_req->value, p_data, len );

        // Change the state to indicate that we are waiting for write response
        le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_WRITE_VALUE;

        status = wiced_bt_gatt_send_write( conn_id, GATT_WRITE, p_write_req );

        WICED_BT_TRACE( "send write. offset %d len %d status %d\n", 0, len, status );

        wiced_bt_free_buffer( p_write_req );
#endif /* BTSTACK_VER */

        if ( status != WICED_BT_SUCCESS )
        {
            le_control_cb.conn[conn_idx].state = LE_CONTROL_STATE_IDLE;
        }
    }
}

#if 0
/*
 * Process notify received over UART.  Return WICED_TRUE if buffer has been
 * queued, WICED_FALSE if buffer has been shipped.
 */
wiced_bool_t hci_control_le_handle_notify( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    wiced_bool_t           command_queued = WICED_FALSE;
    wiced_bt_gatt_status_t rc;

    if ( conn_id > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_id:%04x\n", conn_id );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_id].conn_id != conn_id )
    {
        WICED_BT_TRACE( "no connection:%d\n", le_control_cb.conn[conn_id].conn_id );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
    }
    else if ( handle == 0 )
    {
        WICED_BT_TRACE( "illegal handle:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else if ( !( hci_control_le_data_xfer_ccc & GATT_CLIENT_CONFIG_NOTIFICATION ) )
    {
        WICED_BT_TRACE( "client not registered for notification:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_CLIENT_NOT_REGISTERED );
    }
    else if ( hci_control_le_pending_tx_buffer.tx_buf_saved )
    {
        WICED_BT_TRACE( "client sent next packet before previous is acked:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_IN_PROGRESS );
    }
    else
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved   = WICED_TRUE;
        hci_control_le_pending_tx_buffer.tx_buf_conn_id = conn_id;
        hci_control_le_pending_tx_buffer.tx_buf_type    = HCI_CONTROL_GATT_COMMAND_NOTIFY;
        hci_control_le_pending_tx_buffer.tx_buf_handle  = handle;
        hci_control_le_pending_tx_buffer.tx_buf_len     = len <= sizeof( hci_control_le_pending_tx_buffer.tx_buf_data ) ?
                                                                             len : sizeof( hci_control_le_pending_tx_buffer.tx_buf_data );
        memcpy( hci_control_le_pending_tx_buffer.tx_buf_data, p_data, hci_control_le_pending_tx_buffer.tx_buf_len );

        WICED_BT_TRACE( "notify %d avail bufs %d\n", p_data[0], wiced_bt_ble_get_available_tx_buffers( ) );

        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

        // if there are tx buffers send notification
        if ( wiced_bt_ble_get_available_tx_buffers( ) > 1 )
        {
            if ( ( rc = wiced_bt_gatt_send_notification( conn_id, handle, len, p_data ) ) != WICED_BT_GATT_SUCCESS )
            {
                WICED_BT_TRACE( "Failed to send:%x\n", rc );
            }
            else
            {
                // Notification sent successfully, clear tx_buf_saved flag
                hci_control_le_pending_tx_buffer.tx_buf_saved  = WICED_FALSE;

                hci_control_le_send_write_completed( conn_id, rc );
            }
        }
        else
        {
            WICED_BT_TRACE( "queued conn_id:%04x\n", conn_id );
        }
    }
    return ( hci_control_le_pending_tx_buffer.tx_buf_saved );
}

/*
 * Command received over UART to send Indicate to the peer
 */
void hci_control_le_handle_indicate( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    if ( conn_id > LE_CONTROL_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE( "illegal conn_id:%04x\n", conn_id );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_BAD_HANDLE );
    }
    else if ( le_control_cb.conn[conn_id].state != LE_CONTROL_STATE_IDLE )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_WRONG_STATE );
        WICED_BT_TRACE( "illegal state:%d\n", le_control_cb.conn[conn_id].state );
    }
    else if ( handle == 0 )
    {
        WICED_BT_TRACE( "illegal handle:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else if ( !( hci_control_le_data_xfer_ccc & GATT_CLIENT_CONFIG_INDICATION ) )
    {
        WICED_BT_TRACE( "client not registered for indication:%04x\n", handle );
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_CLIENT_NOT_REGISTERED );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

        wiced_bt_gatt_send_indication( conn_id, handle, len, p_data );
    }
}
#endif

/*
 * Command received over UART to remove the NVRAM stored info and initiate pair
 */
void hci_control_le_handle_delete_nvram_info_and_initiate_pair ( BD_ADDR addr, uint16_t nvram_id )
{
    BD_ADDR                 bda;
    wiced_result_t          result;
    uint8_t                 status;

    STREAM_TO_BDADDR(bda,addr);

    WICED_BT_TRACE( "hci_control_le_handle_delete_nvram_info_and_initiate_pair %02x:%02x:%02x:%02x:%02x:%02x\n",
            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5] );

    hci_control_delete_nvram( nvram_id, WICED_FALSE );
//    wiced_bt_sec_delete_device_record( bda );

//    result = wiced_bt_dev_sec_bond( bda, 0, NULL );
    result = wiced_bt_dev_sec_bond( bda, 0, BT_TRANSPORT_LE, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_dev_sec_bond:%B, result %x\n", bda, result );

    if( ( result == WICED_BT_SUCCESS ) || ( result == WICED_BT_PENDING) )
    {
        status = HCI_CONTROL_STATUS_SUCCESS;
    }
    else
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }

    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, status );
}

/*
 * Command received over UART to get the identity address of the remote device
 */
void hci_control_le_handle_get_identity_address ( BD_ADDR bd_addr )
{
    BD_ADDR identity_addr;

    memset( identity_addr, 0, sizeof( BD_ADDR ) );

    if ( 0) //wiced_bt_get_identity_address( bd_addr, identity_addr ) )
    {
        WICED_BT_TRACE("Identity address is %B\n", identity_addr);
    }
    else
    {
        WICED_BT_TRACE("Identity address not found for this device %B\n", bd_addr);
    }

    wiced_transport_send_data( HCI_CONTROL_LE_EVENT_IDENTITY_ADDRESS, identity_addr, sizeof( BD_ADDR ) );
}

#if (defined(COEX_SUPPORTED) && COEX_SUPPORTED == WICED_TRUE)
/*
 * Command received over UART to set the channel classification for the available 40 channels
 */
static void hci_control_le_handle_set_channel_classification ( uint8_t ble_channel_map[ BTM_BLE_CHNL_MAP_SIZE ] )
{
    WICED_BT_TRACE("Setting LE channel classification [ %02x %02x %02x %02x %02x]\n",
                    ble_channel_map[0], ble_channel_map[1], ble_channel_map[2], ble_channel_map[3], ble_channel_map[4] );

    wiced_bt_ble_set_channel_classification( ble_channel_map );

    hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}
#endif

/*
 * forward received data packet over UART
 */
void hci_control_le_process_data( uint16_t type, uint16_t conn_idx, uint16_t handle, uint8_t *data, int len )
{
    int       i;
    uint8_t   tx_buf [300];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;
    for ( i = 0; i < len; i++ )
    {
        *p++ = data[i];
    }
    wiced_transport_send_data ( type, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer scan event to UART
 */
void hci_control_le_send_scan_state_event( uint8_t status )
{
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_SCAN_STATUS, &status, 1 );
}

/*
 *  transfer advertisement report event to UART
 */
void hci_control_le_send_advertisement_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    int       i;
    uint8_t   tx_buf[300];
    uint8_t   *p = tx_buf;
    uint8_t   len;

    *p++ = p_scan_result->ble_evt_type;
    *p++ = p_scan_result->ble_addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = p_scan_result->remote_bd_addr[5 - i];
    *p++ = p_scan_result->rssi;

#if BTSTACK_VER >= 0x03000001
    // new BTSTACK will send pointer to start of ADV data, and length is the first byte.
    if (( p_adv_data != NULL ) && ( len = *p_adv_data++ ) != 0 )
    {
        if ( p + len + 1 > tx_buf + 255 )
        {
            WICED_BT_TRACE("Bad data\n");
        }
        else
        {
            for( i = 0; i < len; i++ )
            {
                *p++ = *p_adv_data++;
            }
        }
    }
#else
    // currently callback does not pass the data of the adv data, need to go through the data
    // zero len in the LTV means that there is no more data
    while ( ( p_adv_data != NULL ) && ( len = *p_adv_data ) != 0 )
    {
        // In the HCI event all parameters should fit into 255 bytes
        if ( p + len + 1 > tx_buf + 255 )
        {
            WICED_BT_TRACE("Bad data\n");
            break;
        }
        for ( i = 0; i < len + 1; i++ )
            *p++ = *p_adv_data++;
    }
#endif
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer advertise  event to uart
 */
void hci_control_le_send_advertisement_state_event( uint8_t state )
{
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE, &state, 1 );
}

/*
 *  transfer connection event to uart
 */
void hci_control_le_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}


/*
 *  transfer disconnection event to UART
 */
void hci_control_le_send_disconnect_evt( uint8_t reason, uint16_t conn_idx )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = reason;

    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discover complete event to UART
 */
void hci_control_le_send_discover_complete( uint16_t conn_idx )
{
    uint8_t   tx_buf [2];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_DISCOVERY_COMPLETE, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discovery response event to UART
 */
void hci_control_le_send_discovered_service16( uint16_t conn_idx, uint16_t uuid, uint16_t s_handle, uint16_t e_handle )
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = ( uuid >> 8 ) & 0xff;
    *p++ = s_handle & 0xff;
    *p++ = ( s_handle >> 8 ) & 0xff;
    *p++ = e_handle & 0xff;
    *p++ = ( e_handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

void hci_control_le_send_discovered_service128( uint16_t conn_idx, uint8_t *uuid, uint16_t s_handle, uint16_t e_handle )
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    for ( i = 0; i < 16; i++ )
    {
        *p++ = uuid[15 - i];
    }
    *p++ = s_handle & 0xff;
    *p++ = ( s_handle >> 8 ) & 0xff;
    *p++ = e_handle & 0xff;
    *p++ = ( e_handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_SERVICE_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_characteristic16( uint16_t conn_idx, uint16_t char_handle, uint8_t properties, uint16_t handle, uint16_t uuid )
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = char_handle & 0xff;
    *p++ = ( char_handle >> 8 ) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = ( uuid >> 8 ) & 0xff;
    *p++ = properties;
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_characteristic128( uint16_t conn_idx, uint16_t char_handle, uint8_t properties, uint16_t handle, uint8_t *uuid )
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = char_handle & 0xff;
    *p++ = ( char_handle >> 8 ) & 0xff;
    for ( i = 0; i < 16; i++ )
    {
        *p++ = uuid[15 - i];
    }
    *p++ = properties;
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_descriptor16( uint16_t conn_idx, uint16_t handle, uint16_t uuid )
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = uuid & 0xff;
    *p++ = ( uuid >> 8 ) & 0xff;
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_DESCRIPTOR_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer discovery characteristic response event to UART
 */
void hci_control_le_send_discovered_descriptor128( uint16_t conn_idx, uint16_t handle, uint8_t *uuid )
{
    int       i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    for ( i = 0; i < 16; i++ )
    {
        *p++ = uuid[15 - i];
    }
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_CHARACTERISTIC_DISCOVERED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer result of the read procedure to UART
 */
void hci_control_le_send_read_rsp( uint16_t conn_idx, uint8_t *data, int len )
{
    int       i;
    uint8_t   tx_buf [300];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    for ( i = 0; i < len; i++ )
    {
        *p++ = data[i];
    }

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_READ_RESPONSE, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer read request event to UART
 */
void hci_control_le_send_read_req( uint16_t conn_idx, uint16_t handle )
{
    uint8_t   tx_buf[10];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = handle & 0xff;
    *p++ = ( handle >> 8 ) & 0xff;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_READ_REQUEST, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer result of the read procedure to UART
 */
void hci_control_le_send_write_completed( uint16_t conn_idx, uint8_t result )
{
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;
#if BTSTACK_VER >= 0x03000001
    uint16_t conn_id = hci_control_le_get_conn_id(conn_idx);
#else
    uint16_t conn_id = conn_idx;
#endif

    *p++ = conn_id & 0xff;
    *p++ = ( conn_id >> 8 ) & 0xff;
    *p++ = result;

    wiced_transport_send_data ( HCI_CONTROL_GATT_EVENT_WRITE_RESPONSE, tx_buf, ( int )( p - tx_buf ) );
}

#endif /* WICED_APP_LE_INCLUDED */
