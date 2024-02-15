/*
 * spi_task.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

// Board support includes
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ssi.h>

#include <driverlib/udma.h>
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/pin_map.h>
#include <driverlib/ssi.h>
#include <driverlib/gpio.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "priorities.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "spi_task.h"
#include "SSI0.h"

#define SPI_TASK_STACK_SIZE     128  // Stack size in words
#define SPI_ITEM_SIZE           sizeof(uint8_t)
#define SPI_QUEUE_SIZE          5

static bool SSI0received;
static bool SSI0sent;

//xQueueHandle g_pLEDQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

#define SPI_MAX_BUFFER 119
uint8_t rx_buffer[SPI_MAX_BUFFER];
uint8_t tx_buffer[SPI_MAX_BUFFER];

/* The control table used by the uDMA controller.  This table must be aligned
   to a 1024 byte boundary. */
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];

/* RTOS task for handling the SPI interface back to the Beaglebone. */
static void SPI_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t spiTaskDelay;

    spiTaskDelay = 10;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while (1) {


        if (SSI0received == true) {

            SSI0received = false;
            SSI0sent = false;
            //SSI0SendMessage();
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, spiTaskDelay / portTICK_RATE_MS);
    }
}

/* RTOS task initialization */
uint32_t SPI_Task_Init(void) {

    uint8_t i;

    tx_buffer[0] = 0xAA;
    tx_buffer[1] = 0x55;

    for (i = 2; i < SPI_MAX_BUFFER; i++) {
        tx_buffer[i] = i;
    }

    for (i = 0; i < SPI_MAX_BUFFER; i++) {
        rx_buffer[i] = 0x11;
    }

    /* Init the SSI0 device for DMA usage */
    SSI0Init(&SSI0received, &SSI0sent, rx_buffer, tx_buffer, SPI_MAX_BUFFER);

    /* Enable the uDMA controller error interrupt.  This interrupt will occur
       if there is a bus error during a transfer */
    IntEnable(INT_UDMAERR);

    /* Enable the uDMA controller */
    uDMAEnable();

    /* Point at the control table to use for channel control structures */
    uDMAControlBaseSet(pui8ControlTable);

    /* Initialize the uDMA SPI transfers */
    InitSPITransfer();

    /* Create the RTOS task */
    if (xTaskCreate(SPI_Task, (const portCHAR *)"SPI", SPI_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    /* Success */
    return(0);
}
