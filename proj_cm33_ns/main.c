/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Smart I/O I2S controller (TX)
*              in CM33 CPU non-secure environment.
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"

#include "wave.h"
#include "i2s_controller.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Debounce delay for the button */
#define DEBOUNCE_DELAY_MS               (10U)
#define I2S_PRIORITY                    (7UL)
#define I2S_WORD_LENGTH                 (16U)

/* The timeout value in microsecond used to wait for the CM55 core to be booted.
 * Use value 0U for infinite wait till the core is booted successfully.
 */
#define CM55_BOOT_WAIT_TIME_USEC        (10U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR              (CYMEM_CM33_0_m55_nvm_START + \
                                            CYBSP_MCUBOOT_HEADER_SIZE)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* I2S controller configuration */
stc_i2s_controller_config_t i2s_config = 
{
    .word_length = I2S_WORD_LENGTH
};

/* SMARTIO I2S interrupt configuration */
cy_stc_sysint_t smartio_i2s_int_cfg =
{
    .intrSrc          = SPI_I2S_IRQ,
    .intrPriority     = I2S_PRIORITY,
};

/* I2S controller Context */
static stc_i2s_controller_context_t i2s_context;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_app_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void handle_app_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);

    /* Infinite loop */
    while(true);
}

/*******************************************************************************
* Function Name: i2s_interrupt_handler
********************************************************************************
* Summary:
* Interrupt handler for the I2S controller.
*
* Parameters:
*  void
*
* Return:
*  void
*******************************************************************************/
static void i2s_interrupt_handler(void)
{
    i2s_controller_interrupt(&i2s_context);
}

/*******************************************************************************
* Function Name: i2s_callback
********************************************************************************
* Summary:
* I2S callback function. Turns the user LED off.
*
* Parameters:
*  void
*
* Return:
*  void
*******************************************************************************/
static void i2s_callback(void)
{
    /* Turn user LED off */
    Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, CYBSP_LED_STATE_OFF);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*   This is the main function of the CM33 non-secure applicaton.
*   It does...
*    1. Initialize the user button, led and required pwm initialization for 
*       controller clock (CCLK) 
*    2. Initialize SPI and smartio as per configuration structure to act as I2S
*    3. Check for button presses. If pressed, plays the record
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialze TCPWM block with required PWM configuration */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(CYBSP_GENERAL_PURPOSE_TIMER_HW, 
        CYBSP_GENERAL_PURPOSE_TIMER_NUM, 
        &CYBSP_GENERAL_PURPOSE_TIMER_config))
    {
        handle_app_error();
    }

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(CYBSP_GENERAL_PURPOSE_TIMER_HW, CYBSP_GENERAL_PURPOSE_TIMER_NUM);

    /* Start the PWM */
    Cy_TCPWM_TriggerStart_Single(CYBSP_GENERAL_PURPOSE_TIMER_HW, 
                                 CYBSP_GENERAL_PURPOSE_TIMER_NUM);
    

    /* Configure the interrupt for I2S controller */
    Cy_SysInt_Init(&smartio_i2s_int_cfg, &i2s_interrupt_handler);
    NVIC_EnableIRQ(smartio_i2s_int_cfg.intrSrc);

    /* Initialize the I2S controller */
    result = i2s_controller_init(SPI_I2S_HW, CYBSP_SMART_IO_11_HW, 
                            &i2s_config, &i2s_context);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Register the callback for I2S */
    i2s_controller_register_callback(i2s_callback, &i2s_context);

    /* Enable I2S controller */
    result = i2s_controller_enable(&i2s_context);
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Enable CM55. */
    /* CM55_APP_BOOT_ADDR must be updated if CM55 memory layout is changed.*/
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);

    for (;;)
    {
        /* Check if the button was pressed */
        if (CYBSP_BTN_PRESSED == Cy_GPIO_Read(CYBSP_USER_BTN_PORT, 
                                                CYBSP_USER_BTN_PIN))
        {
            
            /* Delay to debounce the button press */
            Cy_SysLib_Delay(DEBOUNCE_DELAY_MS);
            Cy_GPIO_Write(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, 
                            CYBSP_LED_STATE_ON);

            /* Start playing */
            i2s_controller_write(wave_data, WAVE_SIZE, &i2s_context);
        }
    }
}

/* [] END OF FILE */
