#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
//#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "sensor_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define SPI_TASK_STACK_SIZE     128  // Stack size in words
#define SPI_ITEM_SIZE           sizeof(uint8_t)
#define SPI_QUEUE_SIZE          5

bool spiFIFOClear;
xQueueHandle g_pLEDQueue;
extern xSemaphoreHandle g_pUARTSemaphore;

#define SPI_MAX_BUFFER 10
uint8_t buffer[SPI_MAX_BUFFER];
uint8_t bufptr;


/*
 * Interrupt handler for SPI (aka SSI) peripheral.
 */
void SPI_ISR(void) {

    uint8_t i;
    uint8_t count;
    uint32_t interrupt_status;
    uint32_t rxbyte;

    /* Determine which interrupt got us here */
    interrupt_status = SSIIntStatus(SSI0_BASE, true);

    /* Clear all the SPI interrupts */
    SSIIntClear(SSI0_BASE, 0x7F);

    /* Receive data */
    count = 0;
    while (count > 0) {
        count = SSIDataGetNonBlocking(SSI0_BASE, &rxbyte);
    }


    /* Or is the transmit FIFO either half or completely empty? */
    if (interrupt_status & (SSI_TXFF|SSI_TXEOT)) {

        /* Load more message into the FIFO */
        for (i = bufptr; i < SPI_MAX_BUFFER; i++) {

           count = SSIDataPutNonBlocking(SSI0_BASE, buffer[i]);

           /* Exit when TX buffer is full but remember how much of the
            * message was inserted */
           if (count == 0) {
               break;
           } else {
               bufptr += count;
           }
        }

   }
}

/*
 * RTOS task for handling the SPI interface back to the Beaglebone.
 */
static void SPI_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t spiTaskDelay;
    uint8_t i;
    int32_t count;

    spiTaskDelay = 10;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Loop forever.
    while (1) {

        /* Pre-load the buffer with part of the message */
        if (spiFIFOClear) {

            spiFIFOClear = false;
            bufptr = 0;

            for (i = 0; i < SPI_MAX_BUFFER; i++) {
                count = SSIDataPutNonBlocking(SSI0_BASE, buffer[i]);

                /* Exit when TX buffer is full but remember how much of the
                 * message was inserted */
                if (count == 0) {
                    break;
                } else {
                    bufptr += count;
                }
            }

        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, spiTaskDelay / portTICK_RATE_MS);
    }
}

/*
 * RTOS task initialization.
 */
uint32_t SPI_Task_Init(void) {

    spiFIFOClear = false;

    buffer[0] = 0x1F;
    buffer[1] = 0x1E;
    buffer[2] = 0x1D;
    buffer[3] = 0x1C;
    buffer[4] = 0x1B;
    buffer[5] = 0x1A;
    buffer[6] = 0x09;
    buffer[7] = 0x08;
    buffer[8] = 0x07;
    buffer[9] = 0x06;
    bufptr = 0;

    if (xTaskCreate(SPI_Task, (const portCHAR *)"SPI", SPI_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}
