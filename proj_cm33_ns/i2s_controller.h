/*******************************************************************************
* File Name  : i2s_controller.h
*
* Description: This file contains definitions of constants and structures for
*              the I2S controller implementation using the SPI and Smart I/O.
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

#ifndef I2S_CONTROLLER_H_
#define I2S_CONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "cy_pdl.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define I2S_CONTROLLER_FIFO_DEPTH       (64U)
#define SPI_TX_INTERRUPT_MASK           (0U)
#define I2S_FRAME_CHANNEL_WIDTH_8       (8U)
#define I2S_FRAME_CHANNEL_WIDTH_16      (16U)
#define SPI_EMPTY_DATA                  (0x0000U)

/*******************************************************************************
* Enumerated Types
*******************************************************************************/
typedef enum
{
    /* Operation completed successfully */
    I2S_CONTROLLER_SUCCESS,

    /* One ore more input parameters are invalid */
    I2S_CONTROLLER_BAD_PARAM,

    /* Failed to init one of the internal hardware blocks  */
    I2S_CONTROLLER_HW_FAILED,

} en_i2s_controller_status_t;

/*******************************************************************************
* Type Definitions
*******************************************************************************/
typedef void (* cb_i2s_controller_callback_t)(void);

/* Config Structure */
typedef struct
{
    uint32_t word_length;
} stc_i2s_controller_config_t;

/* Context Structure */
typedef struct
{
    /* Constants */
    uint32_t spi_width;
    CySCB_Type *spi_base;
    SMARTIO_PRT_Type *smartio_base;
    cb_i2s_controller_callback_t callback;

    /* Variables */
    volatile size_t tx_length;
    volatile const void *tx_buff;
} stc_i2s_controller_context_t;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
en_i2s_controller_status_t i2s_controller_init(CySCB_Type *spi_base,
                                       SMARTIO_PRT_Type *smartio_base,
                                       stc_i2s_controller_config_t const *config,
                                       stc_i2s_controller_context_t *context);
en_i2s_controller_status_t i2s_controller_deinit(stc_i2s_controller_context_t *context);
en_i2s_controller_status_t i2s_controller_enable(stc_i2s_controller_context_t *context);
en_i2s_controller_status_t i2s_controller_disable(stc_i2s_controller_context_t *context);
void i2s_controller_register_callback(cb_i2s_controller_callback_t callback,
                                 stc_i2s_controller_context_t *context);
bool i2s_controller_is_busy(stc_i2s_controller_context_t *context);
void i2s_controller_clear(stc_i2s_controller_context_t *context);
void i2s_controller_write(const void *frame, size_t length, 
                        stc_i2s_controller_context_t *context);
void i2s_controller_interrupt(stc_i2s_controller_context_t *context);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* I2S_CONTROLLER_H_ */

/* [] END OF FILE */
