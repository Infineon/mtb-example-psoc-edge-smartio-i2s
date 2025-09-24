/*******************************************************************************
* File Name  : i2s_controller.c
*
* Description: This file contains function definitions for implementing the
*              I2S controller interface.
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

#include "i2s_controller.h"
#include "cybsp.h"

/*******************************************************************************
* Function Name: i2s_controller_write_until_full
********************************************************************************
* Summary:
*   Write data to the internal FIFO buffer till full.
*   It handles alignment for 8/16/32 bits.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  void
*
*******************************************************************************/
static void i2s_controller_write_until_full(stc_i2s_controller_context_t *context)
{
    const void *cast_buffer = (const void *)context->tx_buff;  
    int32_t val;

    switch (context->spi_width)
    {
        case I2S_FRAME_CHANNEL_WIDTH_8:
        {
            /* Cast buffer */
            const int8_t *int8_buffer = (const int8_t *)cast_buffer;

            /* Write to the SPI buffer till full */
            while (context->tx_length 
                    && Cy_SCB_SPI_GetNumInTxFifo(context->spi_base) 
                    < (I2S_CONTROLLER_FIFO_DEPTH-1))
            {
                val = ((int16_t)(*int8_buffer)) << 8U;
                Cy_SCB_SPI_Write(context->spi_base, val);
                Cy_SCB_SPI_Write(context->spi_base, SPI_EMPTY_DATA);
                ++int8_buffer;
                --(context->tx_length);
            }

            cast_buffer = (const void *)int8_buffer; 
            break;
        }

        case I2S_FRAME_CHANNEL_WIDTH_16:
        {
            /* Cast buffer */
            const int16_t *int16_buffer = (const int16_t *)cast_buffer;

            /* Write to the SPI buffer till full */
            while (context->tx_length 
                    && Cy_SCB_SPI_GetNumInTxFifo(context->spi_base) 
                    < (I2S_CONTROLLER_FIFO_DEPTH-1))
            {
                val = ((int16_t)(*int16_buffer));
                Cy_SCB_SPI_Write(context->spi_base, val);
                Cy_SCB_SPI_Write(context->spi_base, SPI_EMPTY_DATA);
                ++int16_buffer;
                --(context->tx_length);
            }

            cast_buffer = (const void *)int16_buffer;  
            break;
        }

        default:
        {
            /* Cast buffer */
            const int32_t *int32_buffer = (const int32_t *)cast_buffer;

            /* Write to the SPI buffer till full */
            while (context->tx_length 
                    && Cy_SCB_SPI_GetNumInTxFifo(context->spi_base) 
                    < (I2S_CONTROLLER_FIFO_DEPTH-1))
            {
                val = ((int32_t)(*int32_buffer));
                Cy_SCB_SPI_Write(context->spi_base, CY_HI16(val));
                Cy_SCB_SPI_Write(context->spi_base, CY_LO16(val));
                ++int32_buffer;
                --(context->tx_length);
            }

            cast_buffer = (const void *)int32_buffer; 
            break;
        }
    }

    /* Update the current buffer pointer */
    context->tx_buff = (void *)cast_buffer;
}


/*******************************************************************************
* Function Name: i2s_controller_init
********************************************************************************
* Summary:
*  Initialize the I2S controller block.
*
* Parameters:
*  spi_base     The pointer to the SPI.
*  smartio_base The pointer to the Smart I/O.
*  config       The pointer to the configuration structure
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  en_i2s_controller_status_t
*               Returns the status of this operation. If I2S_CONTROLLER_SUCCESS
*               is not received, the operation failed.
*
*******************************************************************************/
en_i2s_controller_status_t i2s_controller_init(CySCB_Type *spi_base,
                                       SMARTIO_PRT_Type *smartio_base,
                                       stc_i2s_controller_config_t const *config,
                                       stc_i2s_controller_context_t *context)
{
    cy_en_smartio_status_t smartio_status;
    cy_en_scb_spi_status_t spi_status;
    en_i2s_controller_status_t i2s_controller_status = I2S_CONTROLLER_SUCCESS;

    CY_ASSERT(NULL != context);
    CY_ASSERT(NULL != spi_base);
    CY_ASSERT(NULL != smartio_base);

    /* Set variables to the context */
    context->spi_base = spi_base;
    context->smartio_base = smartio_base;
    context->spi_width = config->word_length;

    /* Init the Smart I/O block */
    smartio_status = Cy_SmartIO_Init(smartio_base, &CYBSP_SMART_IO_11_config);
    if (CY_SMARTIO_SUCCESS != smartio_status)
    {
        i2s_controller_status = I2S_CONTROLLER_HW_FAILED;
    }

    /* Init the SPI block */
    spi_status = Cy_SCB_SPI_Init(spi_base, &SPI_I2S_config, NULL);
    if (CY_SCB_SPI_SUCCESS != spi_status)
    {
        i2s_controller_status = I2S_CONTROLLER_HW_FAILED;
    }

    /* Clear callback */
    context->callback = NULL;
    
    return i2s_controller_status;
}

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: i2s_controller_deinit
********************************************************************************
* Summary:
*  De-initialize the I2S controller.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  en_i2s_controller_status_t
*            Returns the status of this operation. If I2S_CONTROLLER_SUCCESS is not
*            received, the operation failed.
*
*******************************************************************************/
en_i2s_controller_status_t i2s_controller_deinit(stc_i2s_controller_context_t *context)
{
    CY_ASSERT(NULL != context);

    /* De-init hardware blocks */
    Cy_SmartIO_Deinit(context->smartio_base);
    Cy_SCB_SPI_DeInit(context->spi_base);

    /* Clear variables in context */
    context->callback = NULL;
    context->spi_base = NULL;
    context->smartio_base = NULL;

    return I2S_CONTROLLER_SUCCESS;
}

/*******************************************************************************
* Function Name: i2s_controller_enable
********************************************************************************
* Summary:
* Enable the I2S controller.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  en_i2s_controller_status_t
*            Returns the status of this operation. If I2S_CONTROLLER_SUCCESS is not
*            received, the operation failed.
*
*******************************************************************************/
en_i2s_controller_status_t i2s_controller_enable(stc_i2s_controller_context_t *context)
{
    CY_ASSERT(NULL != context);

    /* Enable SPI and Smart I/O */
    Cy_SCB_SPI_Enable(context->spi_base);
    Cy_SmartIO_Enable(context->smartio_base);

    return I2S_CONTROLLER_SUCCESS;
}

/*******************************************************************************
* Function Name: i2s_controller_disable
********************************************************************************
* Summary:
* Disable the I2S controller.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  en_i2s_controller_status_t
*            Returns the status of this operation. If I2S_CONTROLLER_SUCCESS is not
*            received, the operation failed.
*
*******************************************************************************/
en_i2s_controller_status_t i2s_controller_disable(stc_i2s_controller_context_t *context)
{
    CY_ASSERT(NULL != context);

    /* Disable SPI and Smart I/O */
    Cy_SmartIO_Disable(context->smartio_base);
    Cy_SCB_SPI_Disable(context->spi_base, NULL);

    return I2S_CONTROLLER_SUCCESS;
}

/*******************************************************************************
* Function Name: i2s_controller_register_callback
********************************************************************************
* Summary:
*  Register the callback to be executed on end of I2S controller transmission.
*
* Parameters:
*  callback    Function to be executed.
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  void
*
*******************************************************************************/
void i2s_controller_register_callback(cb_i2s_controller_callback_t callback,
                                 stc_i2s_controller_context_t *context)
{
    context->callback = callback;
}

/*******************************************************************************
* Function Name: i2s_controller_is_busy
********************************************************************************
* Summary:
*  Return if the I2S controller is transmitting data.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  bool        True if busy, otherwise false.
*
*******************************************************************************/
bool i2s_controller_is_busy(stc_i2s_controller_context_t *context)
{
    return (Cy_SCB_SPI_GetNumInTxFifo(context->spi_base) > 0U);
}

/*******************************************************************************
* Function Name: i2s_controller_clear
********************************************************************************
* Summary:
*  Clear internal FIFO.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  void
*
*******************************************************************************/
void i2s_controller_clear(stc_i2s_controller_context_t *context)
{
    Cy_SCB_SPI_ClearTxFifo(context->spi_base);
}

/*******************************************************************************
* Function Name: i2s_controller_write
********************************************************************************
* Summary:
*  Write data to the I2S TX FIFO. This function is non-blocking.
*
* Parameters:
*  frame       Pointer to the frame to be transmitted.
*  length      Size of the frame.
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
* Return:
*  void
*
*******************************************************************************/
void i2s_controller_write(const void *frame, size_t length, 
                            stc_i2s_controller_context_t *context)
{
    context->tx_buff = frame;
    context->tx_length = length;

    /* Disable SPI TX interrupt */
    Cy_SCB_SetTxInterruptMask(context->spi_base, SPI_TX_INTERRUPT_MASK);
    i2s_controller_write_until_full(context);

    /* Enable the interrupt */
    Cy_SCB_SetTxInterruptMask(context->spi_base, CY_SCB_TX_INTR_LEVEL);
}

/*******************************************************************************
* Function Name: i2s_controller_interrupt
********************************************************************************
* Summary:
*  Interrupt handler for the I2S controller.
*
* Parameters:
*  context      The pointer to the context structure 
*               stc_i2s_controller_context_t allocated by the user. The 
*               structure is used during the I2S controller operation for 
*               internal configuration and data retention. 
*               The user must not modify anything in this structure.
*
* Return:
*  void
*
*******************************************************************************/
void i2s_controller_interrupt(stc_i2s_controller_context_t *context)
{
    /* Disable SPI TX interrupt */
    Cy_SCB_SetTxInterruptMask(context->spi_base, SPI_TX_INTERRUPT_MASK);
    /* Write till full */
    i2s_controller_write_until_full(context);
        
    if (!context->tx_length)
    {
        if (context->callback)
        {
            context->callback();
        }
    }
    else
    {
        /* Enable the interrupt */
        Cy_SCB_SetTxInterruptMask(context->spi_base, CY_SCB_TX_INTR_LEVEL);
    }

    Cy_SCB_ClearTxInterrupt(context->spi_base, CY_SCB_TX_INTR_LEVEL);
}

/* [] END OF FILE */