/*******************************************************************************
* File Name: iso_data_handler.h
*
* Description: This file is the public interface of iso_data_handler.c
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
#ifndef ISO_DATA_HANDLER_H_
#define ISO_DATA_HANDLER_H_

/*******************************************************************************
* INCLUDES
*******************************************************************************/
#include "wiced_bt_cfg.h"

/*******************************************************************************
* Typedefs
*******************************************************************************/
typedef void (*iso_dhm_num_complete_evt_cb_t)(uint16_t cis_handle,
    uint16_t num_sent);
typedef void (*iso_dhm_rx_evt_cb_t)(uint16_t cis_handle, uint8_t *p_data,
    uint32_t length);

/*******************************************************************************
* Function Declarations
*******************************************************************************/
void iso_dhm_init(uint32_t max_sdu_size,
 uint32_t channel_count,uint32_t max_buffers_per_cis,
 iso_dhm_num_complete_evt_cb_t num_complete_cb,
 iso_dhm_rx_evt_cb_t rx_data_cb);
uint8_t *iso_dhm_get_data_buffer(void);
void iso_dhm_free_data_buffer(uint8_t *p_buf);
wiced_bool_t iso_dhm_send_packet(uint16_t psn, uint16_t conn_handle,
    uint8_t ts_flag, uint8_t *p_data_buf, uint32_t data_buf_len);
wiced_bool_t iso_dhm_process_num_completed_pkts(uint8_t *p_buf);
void iso_dhm_process_rx_data(uint8_t *p_data, uint32_t length);
uint32_t iso_dhm_get_header_size(void);


#endif /* ISO_DATA_HANDLER_H_ */
