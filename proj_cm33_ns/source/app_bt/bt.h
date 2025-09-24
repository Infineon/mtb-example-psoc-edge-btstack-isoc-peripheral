/*******************************************************************************
* File Name: bt.h
*
* Description: This is the public interface of bt.c
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef APP_BT_H_
#define APP_BT_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include <stdio.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"

/*******************************************************************************
* Defines
*******************************************************************************/
#define bt_stop_advertisement() \
        bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL)
#define bt_disconnect() wiced_bt_gatt_disconnect( link_conn_id() )
#define bt_get_advertising_mode() wiced_bt_ble_get_current_advert_mode()
#define bt_is_advertising() bt_get_advertising_mode()

/*******************************************************************************
* Function Declarations
*******************************************************************************/
wiced_bt_dev_local_addr_ext_t * dev_info(void);
wiced_result_t bt_init(void);
void bt_enter_pairing(void);
wiced_result_t bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode,
               wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type,
               wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr);

#endif // APP_BT_H_

/* end of file */
