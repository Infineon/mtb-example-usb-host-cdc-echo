/*********************************************************************************
 * File Name        :   main.c
 *
 * Description      :   This is the source code for USB Host CDC Code Example
 *                      for ModusToolbox. 
 *
 * Related Document :   See README.md
 *
*******************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 **********************************************************************************/

/* MTB header file includes*/
#include "cy_retarget_io.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_pdl.h"

/* FreeRTOS header file */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* emUSB-Host header file includes */
#include "USBH.h"
#include "USBH_CDC.h"

/***********************************************************************************
 *  Define configurables
 **********************************************************************************/
#define USB_MAIN_TASK_MEMORY_REQ    (500U)
#define USB_ISR_TASK_MEMORY_REQ     (500U)
#define PRINT_TASK_MEMORY_REQ       (500U)
#define WAIT_COUNT_VALUE            (10U)
#define DELAY_TASK                  (100U)
#define DELAY_ECHO_COMMUNICATION    (5000U)

/***********************************************************************************
 *  Forward Declaration
 **********************************************************************************/
static void usb_device_notify(void* usb_context, uint8_t usb_index, USBH_DEVICE_EVENT usb_event);
static void usbh_task(void* arg);
static void usbh_isr_task(void* arg);
static void device_task(void);
static void usbh_cdc_task(void* arg);   

/***********************************************************************************
 *   Global Variables
 **********************************************************************************/
static USBH_NOTIFICATION_HOOK       usbh_cdc_notification;
static volatile uint8_t             device_ready;
static volatile int8_t              device_index;
static uint8_t                      data_buffer[64];
static uint8_t                      packet_counter;


/***********************************************************************************
 *  Function Name: main
 ***********************************************************************************
 * Summary:
 * This is the main function for CM4 CPU. 
 *    1. Initializes the device and board peripherals.
 *    2. Initializes other essential ports and peripherals.
 *    3. Initializes the RTOS handle for usbh_cdc_task.
 *    4. Starts the RTOS task scheduler.  
 *
 * Parameters:
 * void
 *
 * Return:
 * int
 *
 **********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t rtos_task_status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_LED_RGB_BLUE, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, 0U);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf( "====================================================================="
                "============================\r\n\n\t\t\t\t"
             "emUSB Host: CDC Echo application\r\n\n"
             "====================================================================="
                "============================\r\n\n");


    rtos_task_status = xTaskCreate(usbh_cdc_task, "usbh_cdc_task", 1500U, NULL, 
                                   configMAX_PRIORITIES - 1, NULL);
    if (rtos_task_status != pdPASS)
    {
        CY_ASSERT(0);
    }


    vTaskStartScheduler();
    printf("APP_LOG: Error: FreeRTOS doesn't start\r\n");
    for (;;)
    {
    }
}

/***********************************************************************************
 *  Function Name: usbh_cdc_task
 ***********************************************************************************
 * Summary:
 * Initializes emUSB-Host stack and registers all necessary tasks.
 *
 * Parameters:
 * arg - is not used in this function, is required by FreeRTOS
 * 
 * Return:
 * void
 *
 **********************************************************************************/
static void usbh_cdc_task(void* arg)
{
    (void)arg;

    USBH_STATUS usb_status;
    BaseType_t rtos_task_status;

    uint32_t wait_counter = WAIT_COUNT_VALUE;

    printf("APP_LOG: Start USB configuration \r\n");

    /* Initialize USBH stack */
    USBH_Init();

    /* Create two tasks which are mandatory for USBH operation */

    printf("APP_LOG: Register usbh_task task \r\n");
    rtos_task_status = xTaskCreate(usbh_task, "usbh_task",
                                   USB_MAIN_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 1, NULL);

    if (rtos_task_status != pdPASS)
    {
        CY_ASSERT(0);
    }

    printf("APP_LOG: Register usbh_isr_task task \r\n\n");
    rtos_task_status = xTaskCreate(usbh_isr_task, "usbh_isr_task",
                                   USB_ISR_TASK_MEMORY_REQ, NULL, 
                                   configMAX_PRIORITIES - 2, NULL);

    if (rtos_task_status != pdPASS)
    {
        CY_ASSERT(0);
    }

    printf("APP_LOG: Initialize CDC classes \r\n\n");

    /* Initialize CDC classes */
    USBH_CDC_Init();

    USBH_CDC_SetConfigFlags(USBH_CDC_IGNORE_INT_EP | USBH_CDC_DISABLE_INTERFACE_CHECK);
    usb_status = USBH_CDC_AddNotification(&usbh_cdc_notification, usb_device_notify, NULL);

    if (usb_status != USBH_STATUS_SUCCESS)
    {
        CY_ASSERT(0);
    }

    printf("APP_LOG: Waiting for a USB CDC device \r\n\n");

    for (;;)
    {

        if (wait_counter == 0)
        {
            wait_counter = WAIT_COUNT_VALUE;
            /* Wait State */
            cyhal_gpio_toggle(CYBSP_LED_RGB_BLUE);
        }
        else
        {
            wait_counter --;
        }

        vTaskDelay(DELAY_TASK);
        if (device_ready)
        {
            device_task();
        }
    }
}


/***********************************************************************************
 *  Function Name: usbh_task
 ***********************************************************************************
 * Summary:
 * The function is responsibe for calling USBH_Task(). During the execution of the
 * program, exit from the infinite loop is not expected. 
 *
 * Parameters:
 * arg - is not used in this function, is required by FreeRTOS
 * 
 * Return:
 * void
 *
 **********************************************************************************/
static void usbh_task(void* arg)
{
    (void)arg;

    while(1)
    {
        USBH_Task();
    }
}

/***********************************************************************************
 *   Function Name: usbh_isr_task
 ***********************************************************************************
 * Summary:
 * The function is responsibe for calling USBH_ISRTask(). During the execution of 
 * the program, exit from the infinite loop is not expected. 
 *
 * Parameters:
 * arg - is not used in this function, is required by FreeRTOS
 * 
 * Return:
 * void
 *
 **********************************************************************************/
static void usbh_isr_task(void* arg)
{
    (void)arg;

    while(1)
    {
        USBH_ISRTask();
    }
}


/***********************************************************************************
 * Function Name: usb_device_notify
 ***********************************************************************************
 * Summary:
 * This function registers the addition or removal of devices from the host.
 * 
 * Parameters:
 * usb_context  :   Pointer to a context passed by the user in the call to one of
 *                  the register functions.
 * usb_index    :   Zero based index of the device that was added or removed.
 *                  First device has index 0, second one has index 1, etc
 * usb_event    :   Enum USBH_DEVICE_EVENT which gives information about the 
 *                  event that occurred.
 * 
 * Return:
 * void
 *
 **********************************************************************************/
static void usb_device_notify(void* usb_context, uint8_t usb_index, USBH_DEVICE_EVENT usb_event)
{
    (void)usb_context;

    switch (usb_event)
    {
        case USBH_DEVICE_EVENT_ADD:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Device added [%d]" 
                                  "========================\n\n\n\n", usb_index);
            device_index    = usb_index;
            device_ready    = 1;
            packet_counter  = 0;
            break;
        }

        case USBH_DEVICE_EVENT_REMOVE:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Device removed [%d]"
                                  "======================\n\n\n\n", usb_index);
            device_ready    = 0;
            device_index    = -1;
            break;
        }

        default:
        {
            printf("\n\n");
            USBH_Logf_Application("======================== APP_LOG: Invalid event [%d]"
                                  "=======================\n\n\n\n", usb_index);
            break;
        }
    }
}

/***********************************************************************************
 * Function Name: device_task
 ***********************************************************************************
 * Summary:
 * This function is responsible for retreival of the device information and 
 * configures the CDC device to start the echo communication. It contains the 
 * print logs for the data packets as well as the echo data stream. 
 * 
 * Parameters:
 * void
 * 
 * Return:
 * void
 *
 **********************************************************************************/
static void device_task(void)
{
    /* Open the device, the device index is retrieved from the notification callback. */
    USBH_CDC_HANDLE             device_handle = USBH_CDC_Open(device_index);

    if (device_handle)
    {
        USBH_CDC_DEVICE_INFO    usb_device_info;
        USBH_STATUS             usb_status;
        unsigned long           numBytes;

        /* Configure the CDC device. */
        USBH_CDC_SetTimeouts(device_handle, 50, 50);
        USBH_CDC_AllowShortRead(device_handle, 1);
        USBH_CDC_SetCommParas(device_handle, USBH_CDC_BAUD_115200, USBH_CDC_BITS_8,
                                USBH_CDC_STOP_BITS_1, USBH_CDC_PARITY_NONE);

        /* Retrieve the information about the CDC device */ 
        USBH_CDC_GetDeviceInfo(device_handle, &usb_device_info);  

        printf("Initiating echo communication: Packet Number: %u\n", packet_counter);  

        printf("Vendor  ID = 0x%.4X\n", usb_device_info.VendorId);
        printf("Product ID = 0x%.4X\n", usb_device_info.ProductId);

        printf("Writing to the device \"Hello Infineon!\"\n");
        USBH_CDC_Write(device_handle, (const uint8_t *)"Hello Infineon!\n", 16U, &numBytes);
        printf("Reading from the device\n");
        usb_status = USBH_CDC_Read(device_handle, data_buffer, sizeof(data_buffer), &numBytes);        

        if (usb_status != USBH_STATUS_SUCCESS)
        {
            printf("Error occurred during reading from device\n");
        }
        else
        {
            data_buffer[numBytes] = 0;
            printf("Received: %s \n",(char *)data_buffer);
            printf("Communication of USB Host with USB Device Successful\n");
        }
        
        printf("Re-initiating echo communication in 5 seconds\n\n\n\n");
        cyhal_system_delay_ms(DELAY_ECHO_COMMUNICATION);
        packet_counter++;
    }
}


/* [] END OF FILE */