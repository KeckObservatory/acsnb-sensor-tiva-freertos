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
bool SSI_Load_Message(void) {

    uint8_t i;
    uint8_t checksum;
    uint32_t ssi0_state;

    /* Is the SSI0 chip select active?  If so, don't attempt to load the next message
     * if we are in the middle of transmitting it */
    ssi0_state = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
    if (ssi0_state == 0) {
        return false;
    }

    /* Lock the structure with the message, by taking the semaphore */
    if (xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY) == pdTRUE) {

        /* We got the lock and can now work on the buffers */

        /* Copy the received message so it can be parsed */
        memcpy(rx_message.buf, rx_message_in.buf, SSI_MESSAGE_LENGTH);

        /* Create the header for the outbound message */
        tx_message_raw.msg.size = SSI_MESSAGE_LENGTH;
        tx_message_raw.msg.version0 = FIRMWARE_REV_0;
        tx_message_raw.msg.version1 = FIRMWARE_REV_1;
        tx_message_raw.msg.version2 = FIRMWARE_REV_2;

        /* Update the RTOS tick value for the next outbound message, as
         * a heartbeat function. */
        tx_message_raw.msg.tick_count = xTaskGetTickCount();

        /* Start with a checksum of 0 before we calculate the real value, so
         * as to not affect the calculation! */
        tx_message_raw.msg.checksum = 0;

        /* Set the checksum in the outbound message */
        for (i = 0, checksum = 0; i < SSI_MESSAGE_LENGTH; i++) {
            checksum += tx_message_raw.buf[i];
        }

        /* Take the 2's complement of the sum and put it back in the message */
        tx_message_raw.msg.checksum = ~checksum + 1;

        /* Copy the next outbound message to the storage the SSI+DMA will
         * use to transmit it */
        memcpy(tx_message_out_p, tx_message_raw_p, SSI_MESSAGE_LENGTH);

        /* Release the semaphore */
        xSemaphoreGive(g_txMessageSemaphore);

    } else {

        /* Could not get the lock, return and try again later */
        return false;
    }

    /* If the semaphore is not obtained, something terrible has happened; in
     * that case the heartbeat will not be updating and the Beaglebone code
     * will determine the course of action to recover, probably by power
     * cycling the TIVA. */

    /* If we got this far, everything worked */
    return true;
}


/* -----------------------------------------------------------------------------
 * Decodes the last message from the Beaglebone.
 */
void SSI_Decode_Message_In(void) {

    uint8_t i;
    uint8_t checksum;

    /* Calculate the checksum of the inbound message */
    for (i = 0, checksum = 0; i < SSI_MESSAGE_LENGTH; i++) {
        checksum += rx_message.buf[i];
    }

    /* A valid checksum calculation results in 0x00 */
    if (checksum == 0) {

        /* Check the message key.  This will prevent an all 0's message from
         * being parsed. */
        if (rx_message.msg.key == RX_MESSAGE_KEY) {

            /* Now all the fields can take effect! */

            /* Sensor enables (boolean) */
            sensor_control[SENSOR1].enabled = (bool) rx_message.msg.enable_sensor1;
            sensor_control[SENSOR2].enabled = (bool) rx_message.msg.enable_sensor2;
            sensor_control[SENSOR3].enabled = (bool) rx_message.msg.enable_sensor3;
            sensor_control[SENSOR4].enabled = (bool) rx_message.msg.enable_sensor4;
            sensor_control[SENSOR5].enabled = (bool) rx_message.msg.enable_sensor5;
            sensor_control[SENSOR6].enabled = (bool) rx_message.msg.enable_sensor6;

            /* Single ended sensing enables (boolean) */
            sensor_control[SENSOR1].enable_c1_c2 = (bool) rx_message.msg.enable_single1;
            sensor_control[SENSOR2].enable_c1_c2 = (bool) rx_message.msg.enable_single2;
            sensor_control[SENSOR3].enable_c1_c2 = (bool) rx_message.msg.enable_single3;
            sensor_control[SENSOR4].enable_c1_c2 = (bool) rx_message.msg.enable_single4;
            sensor_control[SENSOR5].enable_c1_c2 = (bool) rx_message.msg.enable_single5;
            sensor_control[SENSOR6].enable_c1_c2 = (bool) rx_message.msg.enable_single6;

            /* Relay settings (relay_position_t; 0 = LBL, 1 = Kona) */
            sensor_control[SENSOR1].relay_position = (relay_position_t) rx_message.msg.relay1;
            sensor_control[SENSOR2].relay_position = (relay_position_t) rx_message.msg.relay2;
            sensor_control[SENSOR3].relay_position = (relay_position_t) rx_message.msg.relay3;
            sensor_control[SENSOR4].relay_position = (relay_position_t) rx_message.msg.relay4;
            sensor_control[SENSOR5].relay_position = (relay_position_t) rx_message.msg.relay5;
            sensor_control[SENSOR6].relay_position = (relay_position_t) rx_message.msg.relay6;
        }
    }
}


/* -----------------------------------------------------------------------------
 * SSI RTOS task main loop.  This task handles the SSI interface back to the
 * Beaglebone.
 *
 * Runs and never returns.
 */
static void SSI_Task(void *pvParameters) {

    portTickType wake_time;
    uint32_t host_timer = 0;
    bool refresh = false;
    volatile uint32_t now = 0;
    bool loaded;

    /* Delay 10ms per execution of the loop */
    uint32_t task_delay = 10;

    /* Get the current tick count */
    wake_time = xTaskGetTickCount();

    /* Loop forever */
    while (1) {

        now = xTaskGetTickCount();

        /* Time how long it's been since the Beaglebone talked to us */
        host_timer += task_delay;

        if (host_timer > 2000) {

            /* Force a fresh message for the SPI */
            refresh = true;
            host_timer = 0;
        }

        /* Message has been received, prep the next one */
        if (rxtx_message_ready || refresh) {

            /* Decode the inbound message */
            SSI_Decode_Message_In();

            /* Load the next message */
            loaded = SSI_Load_Message();

            /* Clear the message ready flag, but only if we loaded a new message */
            if (loaded) {
                refresh = false;
                rxtx_message_ready = false;
            }
        }

        /* Wait for the required amount of time */
        vTaskDelayUntil(&wake_time, task_delay / portTICK_RATE_MS);
        //vTaskDelay(task_delay / portTICK_RATE_MS);
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
    SSI0Init();

    /* Enable the uDMA controller error interrupt.  This interrupt will occur
       if there is a bus error during a transfer */
    IntEnable(INT_UDMAERR);

    /* Enable the uDMA controller */
    uDMAEnable();

    /* Point at the control table to use for channel control structures */
    uDMAControlBaseSet(pui8ControlTable);

    /* Initialize the first uDMA SSI transfer */
    SSI0InitTransfer();

    /* Create the RTOS task */
    if (xTaskCreate(SSI_Task, (const portCHAR *)"SSI", SSI_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SSI_TASK, NULL) != pdTRUE) {
        return (1);
    }

    /* Success */
    return (0);
}
