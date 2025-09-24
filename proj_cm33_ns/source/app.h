/*******************************************************************************
* File Name: app.h
*
* Description: This file is the public interface of app.c
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
#ifndef APP_H_
#define APP_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "bt.h"
#include "gatt.h"
#include "isoc_peripheral.h"
#include "link.h"
#include "button.h"
#include "cyabs_rtos_impl.h"

/*******************************************************************************
* Defines
*******************************************************************************/
/* Priority for GPIO Button Interrupt */
#define GPIO_INTERRUPT_PRIORITY     (7U)
#define ISO_SDU_SIZE                (100U)
#define cfg_mtu()                   (cy_bt_cfg_ble.ble_max_rx_pdu_size)

/*******************************************************************************
* Function Declarations
*******************************************************************************/
void app_remove_host_bonding(void);
wiced_bt_gatt_status_t app_gatt_write_handler( uint16_t conn_id,
                           wiced_bt_gatt_write_req_t * p_wr_data );
wiced_bt_gatt_status_t app_gatt_read_req_handler(uint16_t conn_id,
                           wiced_bt_gatt_read_t *p_req,
                           wiced_bt_gatt_opcode_t opcode,
                           uint16_t len_requested );
void app_link_up(wiced_bt_gatt_connection_status_t * p_status);
void app_link_down(const wiced_bt_gatt_connection_status_t * p_status);
void app_adv_state_changed(wiced_bt_ble_advert_mode_t old_adv,
                           wiced_bt_ble_advert_mode_t adv);
wiced_result_t app_init(void);
void application_start( void );

#endif // APP_H_
