/*
 * spi_task.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define SPI_TASK_C_
#include "includes.h"

#define SPI_TASK_STACK_SIZE     128  // Stack size in words
#define SPI_ITEM_SIZE           sizeof(uint8_t)
#define SPI_QUEUE_SIZE          5

/* -----------------------------------------------------------------------------
 * RTOS task for handling the SPI interface back to the Beaglebone.
 */
static void SPI_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t spiTaskDelay;

    spiTaskDelay = 10;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while (1) {

        /* Message has been received, prep the next one */
        if (rxMessageReady) {

        }

        if (rxMessageReady) {

            rxMessageReady = false;
            //SSI0sent = false;
            //SSI0SendMessage();
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, spiTaskDelay / portTICK_RATE_MS);
    }
}

/* -----------------------------------------------------------------------------
 * RTOS task initialization.
 */
uint32_t SPI_Task_Init(void) {

    uint8_t i;

    txMessage.buf[0] = 0xAA;
    txMessage.buf[1] = 0x55;

    for (i = 2; i < TX_MESSAGE_LENGTH; i++) {
        txMessage.buf[i] = i;
    }

    for (i = 0; i < TX_MESSAGE_LENGTH; i++) {
        rxMessage.buf[i] = 0x11;
    }


    txMessageReady = true;

    /* Init the SSI0 device for DMA usage */
    SSI0Init(rxMessage.buf, txMessage.buf, TX_MESSAGE_LENGTH);

    /* Enable the uDMA controller error interrupt.  This interrupt will occur
       if there is a bus error during a transfer */
    IntEnable(INT_UDMAERR);

    /* Enable the uDMA controller */
    uDMAEnable();

    /* Point at the control table to use for channel control structures */
    uDMAControlBaseSet(pui8ControlTable);

    /* Initialize the uDMA SPI transfers */
    SSI0InitTransfer();

    /* Create the RTOS task */
    if (xTaskCreate(SPI_Task, (const portCHAR *)"SPI", SPI_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    /* Success */
    return(0);
}
