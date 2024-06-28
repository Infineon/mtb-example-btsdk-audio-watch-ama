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
 * This file implement BTLE application controlled over UART for AMA service.
 * The GATT database is defined in this file and is not changed by the MCU.
 *
 */
#include <wiced_bt_dev.h>
#include <wiced_memory.h>
#include <wiced_transport.h>
#include <hci_control_api.h>
#include <wiced_bt_ama_ble.h>
#include "GeneratedSource/cycfg_gatt_db.h"
#include "hci_control.h"
#include "hci_control_le.h"
#include "hci_control_le_ama.h"
#include "le_peripheral.h"

/******************************************************
 *                     Constants
 ******************************************************/

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern hci_control_le_cb_t le_control_cb;
extern wiced_timer_t hci_control_le_connect_timer;

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Enable LE Control
 */
void hci_control_le_ama_enable(const wiced_bt_cfg_settings_t *settings)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_bt_ble_advert_elem_t advert_elem = {
        .p_data = app_gap_device_name,
        .len = app_gap_device_name_len,
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,
    };
    wiced_bt_ama_ble_conf_t ama_ble_conf = {
        .app_settings = settings,
        .vendor_id = CYPRESS_COMPANY_ID,
        .color = 1,
        .p_gatt_cb = hci_control_le_gatt_callback,
        .gatt_db_handle = {
            .tx_val = HDLC_ALEXA_SERVICE_ALEXA_CHARACTERISTIC_TX_VALUE,
            .rx_val = HDLC_ALEXA_SERVICE_ALEXA_CHARACTERISTIC_RX_VALUE,
            .rx_cfg_desc = HDLD_ALEXA_SERVICE_ALEXA_CHARACTERISTIC_RX_CLIENT_CHAR_CONFIG,
        },
        .appended_adv_data = {
            .elem_num = 1,
            .p_elem = &advert_elem,
        },
    };

    WICED_BT_TRACE("%s\n", __func__);

    /*  GATT DB Initialization */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    WICED_BT_TRACE("wiced_bt_gatt_db_init 0x%02x\n", gatt_status);

    if (!wiced_bt_ama_ble_init(&ama_ble_conf))
    {
        WICED_BT_TRACE("ERROR wiced_bt_ama_ble_init\n");
    }

    /* Initialize connection timer */
    wiced_init_timer(&hci_control_le_connect_timer,
            &hci_control_le_connect_timeout, 0, WICED_SECONDS_TIMER);
}
