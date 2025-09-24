/*******************************************************************************
* File Name: link.h
*
* Description: This file is the public interface of link.c
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
*******************************************************************************/
#ifndef LINK_H__
#define LINK_H__

/*******************************************************************************
* INCLUDES
******************************************************************************/
#include "wiced_bt_gatt.h"

/*******************************************************************************
 * DEFINES
 ******************************************************************************/
#define BT_TRANSPORT_NONE                      (0U)
#define NON_ISOC_ACL_CONN_INTERVAL             (6U)
#define NON_ISOC_ACL_LINK_SUPERVISION_TIMEOUT  (200U)     /* 2 sec timeout */
#define ISOC_ACL_LINK_SUPERVISION_TIMEOUT      (1500U)     /* 15 sec timeout */

/*******************************************************************************
* FUNCTION DECLARATIONS
*******************************************************************************/
uint8_t link_transport(void);
uint16_t link_conn_id(void);
uint16_t link_acl_conn_handle(void);
uint16_t link_first_acl_conn_handle(void);
void link_set_encrypted(wiced_bool_t set);
wiced_bool_t link_is_encrypted(void);
void link_set_parameter_updated(wiced_bool_t set);
wiced_bool_t link_is_parameter_updated(void);
void link_set_indication_pending(wiced_bool_t set);
wiced_bool_t link_is_indication_pending(void);
void link_set_bonded(wiced_bool_t set);
wiced_bool_t link_is_bonded(void);
wiced_bt_gatt_status_t link_up(
        wiced_bt_gatt_connection_status_t * p_status );
wiced_bt_gatt_status_t link_down(const
wiced_bt_gatt_connection_status_t * p_status );
wiced_bool_t link_is_connected(void);
wiced_bt_gatt_connection_status_t * link_connection_status(void);
wiced_bool_t link_set_acl_conn_interval(uint16_t interval);

#endif // LINK_H__

/* end of file */
