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
 * A2DP support for AV Sink application
 */
#ifndef A2DP_SINK_H
#define A2DP_SINK_H

#include "bt_types.h"

#define CASE_RETURN_STR(const) case const: return #const;

typedef enum
{
    AV_SNK_STATE_IDLE,              /* Initial state (channel is unused) */
    AV_SNK_STATE_CONFIGURED,        /* Remote has sent configuration request */
    AV_SNK_STATE_CONNECTED,         /* Signaling Channel is connected and active */
    AV_SNK_STATE_STARTED,			/* Data streaming */
} AV_SNK_STATE;

typedef enum
{
    AV_SNK_STREAM_STATE_STOPPED,
    AV_SNK_STREAM_STATE_STARTING,
    AV_SNK_STREAM_STATE_STARTED,
    AV_SNK_STREAM_STATE_STOPPING
} AV_SNK_STREAM_STATE;

#ifdef USE_AVRCP_CTRLR
#define A2DP_SINK_SDP_DB_SIZE 200
#else
#define A2DP_SINK_SDP_DB_SIZE 81
#endif

extern void a2dp_sink_app_init( void );
extern wiced_result_t a2dp_sink_app_deinit( void );
extern void a2dp_sink_mute_unmute_audio( void );
void a2dp_sink_am_stream_suspend(void);
void a2dp_sink_am_stream_resume(void);

#endif  /* A2DP_SINK_H */
