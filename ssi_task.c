/*
 * ssi_task.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define SSI_TASK_C_
#include "includes.h"

#define SSI_TASK_STACK_SIZE     128  // Stack size in words
#define SSI_ITEM_SIZE           sizeof(uint8_t)
#define SSI_QUEUE_SIZE          5


/* -----------------------------------------------------------------------------
 * Prepares the next message back to the Beaglebone.
 */
void SSI_Load_Message(void) {

    uint8_t i;
    uint8_t checksum;
    bool result = (bool) pdTRUE;

    /* Lock the structure with the message, by taking the semaphore
       but do not wait forever */
    result = xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY);

    if (result) {

        /* We got the lock and can now work on the buffers */

        /* Copy the received message so it can be parsed */
        memcpy(rx_message.buf, rx_message_in.buf, SSI_MESSAGE_LENGTH);

        /* Create the header for the outbound message */
        tx_message.msg.size = SSI_MESSAGE_LENGTH;
        tx_message.msg.version0 = FIRMWARE_REV_0;
        tx_message.msg.version1 = FIRMWARE_REV_1;
        tx_message.msg.version2 = FIRMWARE_REV_2;

        /* Update the RTOS tick value for the next outbound message, as
         * a heartbeat function. */
        tx_message.msg.tick_count = xTaskGetTickCount();


        //OTHER STUFF

        /* Start with a checksum of 0 before we calculate the real value, so
         * as to not affect the calculation! */
        tx_message.msg.checksum = 0;

        /* Set the checksum in the outbound message */
        for (i = 0, checksum = 0; i < SSI_MESSAGE_LENGTH; i++)
            checksum += tx_message.buf[i];

        /* Take the 2's complement of the sum and put it back in the message */
        tx_message.msg.checksum = ~checksum + 1;

        /* Copy the next outbound message to the storage the SSI+DMA will
         * use to transmit it */
        memcpy(tx_message_out.buf, tx_message.buf, SSI_MESSAGE_LENGTH);

        /* Release the semaphore */
        xSemaphoreGive(g_txMessageSemaphore);
    }

    /* If the semaphore is not obtained, something terrible has happened; in
     * that case the heartbeat will not be updating and the Beaglebone code
     * will determine the course of action to recover, probably by power
     * cyling the TIVA. */
}


/* -----------------------------------------------------------------------------
 * SSI RTOS task main loop.  This task handles the SSI interface back to the
 * Beaglebone.
 *
 * Runs and never returns.
 */
static void SSI_Task(void *pvParameters) {

    portTickType wake_time;
    uint32_t host_timer;
    bool refresh = false;

    /* Delay 10ms per execution of the loop */
    uint32_t task_delay = 10;

    /* Get the current tick count */
    wake_time = xTaskGetTickCount();

    /* Loop forever */
    while (1) {

        /* Time how long it's been since the Beaglebone talked to us */
        host_timer += task_delay;

        if (host_timer > 1000) {

            /* Force a fresh message for the SPI */
            refresh = true;
            host_timer = 0;
        }

        /* Message has been received, prep the next one */
        //if (rxtx_message_ready || refresh) {
        if (refresh) {

            refresh = false;

            /* Load the next message */
            SSI_Load_Message();

            /* Clear the message ready flag */
            rxtx_message_ready = false;
        }

        /* Wait for the required amount of time */
        vTaskDelayUntil(&wake_time, task_delay / portTICK_RATE_MS);
    }
}

/* -----------------------------------------------------------------------------
 * SSI RTOS task initialization, runs once at startup.
 */
uint32_t SSI_Task_Init(void) {

    /* Start off without a message ready to process */
    rxtx_message_ready = false;

    /* Load the first message, basically just the version info and tick count */
    SSI_Load_Message();

    /* Init the SSI0 device for DMA usage */
    SSI0Init(rx_message.buf, tx_message_out.buf, SSI_MESSAGE_LENGTH, &rxtx_message_ready);

    /* Enable the uDMA controller error interrupt.  This interrupt will occur
       if there is a bus error during a transfer */
    IntEnable(INT_UDMAERR);

    /* Enable the uDMA controller */
    uDMAEnable();

    /* Point at the control table to use for channel control structures */
    uDMAControlBaseSet(pui8ControlTable);

    /* Initialize the uDMA SSI transfers */
    SSI0InitTransfer();

    /* Create the RTOS task */
    if (xTaskCreate(SSI_Task, (const portCHAR *)"SSI", SSI_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SSI_TASK, NULL) != pdTRUE) {
        return(1);
    }

    /* Success */
    return(0);
}
