#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "sensor_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define SENSOR_TASK_STACK_SIZE     128  // Stack size in words
#define SENSOR_ITEM_SIZE           sizeof(uint8_t)
#define SENSOR_QUEUE_SIZE          5

xQueueHandle g_pLEDQueue;

extern xSemaphoreHandle g_pUARTSemaphore;

static void Sensor_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t sensorTaskDelay;
    int32_t heartbeatTimer;

    sensorTaskDelay = 250;
    bool toggle;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Count down 500ms for each heartbeat LED change.  Tick rate is 1KHz, which is 1ms per tick.
    heartbeatTimer = 500;

    // Loop forever.
    while (1) {

        /* This will be invoked once per millisecond */
        heartbeatTimer--;

        if (heartbeatTimer < 0) {

            if (toggle) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // off
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // off
            }

            toggle = !toggle;

            heartbeatTimer = 500;
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, sensorTaskDelay / portTICK_RATE_MS);
    }
}

uint32_t Sensor_Task_Init(void) {

    if(xTaskCreate(Sensor_Task, (const portCHAR *)"SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}
