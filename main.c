/*
 * main.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define MAIN_C_
#include "includes.h"


/*
 * The error routine that is called if the driver library encounters an error.
 */
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line) { }
#endif

/* -----------------------------------------------------------------------------
 * This hook is called by FreeRTOS when an stack overflow error is detected.
 */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {

    /* This function can not return, so loop forever.  Interrupts are disabled
     * on entry to this function, so no processor interrupts will interrupt
     * this loop.
     */
    while(1) {}
}

/* -----------------------------------------------------------------------------
 * Configure the virtual UART and its pins.  This must be called before UARTprintf().
 */
void ConfigureUART(void) {

#ifdef VIRTUAL_UART_SUPPORT

    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART01
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

#endif
}


/* -----------------------------------------------------------------------------
 * A printf() function for the virtual serial port that uses the UART semaphore
 * to safely control access to the port.
 */
void v_printf(const char *pcString, ...) {

#ifdef VIRTUAL_UART_SUPPORT
    va_list vaArgP;

    /* Take the semaphore but do not wait forever */
    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);

    /* Start the varargs processing */
    va_start(vaArgP, pcString);

    UARTvprintf(pcString, vaArgP);

    /* We're finished with the varargs now */
    va_end(vaArgP);

    /* Release the semaphore */
    xSemaphoreGive(g_pUARTSemaphore);
#endif
}


/* -----------------------------------------------------------------------------
 * API for timers.
 */

/* Initialize a timer, and start it */
void timer_set(timer_t* timer, uint32_t interval) {
    timer->interval = interval;
    timer_start(timer);
}

/* Start a timer */
void timer_start(timer_t* timer) {
    timer->start_time = xTaskGetTickCount();
}

/*  Returns true if the timer has expired */
bool timer_expired(timer_t* timer) {
    return (xTaskGetTickCount() >= (timer->start_time + timer->interval));
}

/* Continue a timer, retaining the base time */
void timer_continue(timer_t* timer) {
    timer->start_time += timer->interval;
}

/* Expire a timer */
void timer_expire(timer_t* timer) {
    timer->start_time = 0;
}

/* Reports back the amount of time remaining before the timer is considered
 * Done, in milliseconds */
uint32_t timer_time_remaining(timer_t* timer) {

  uint32_t clock = xTaskGetTickCount();

  if (clock >= (timer->start_time + timer->interval))
    return 0;
  else
    return (timer->start_time + timer->interval) - clock;
}

/* Reports back the amount of time elapsed on the timer, in milliseconds */
uint32_t timer_elapsed_time(timer_t* timer) {
  return (xTaskGetTickCount() - timer->start_time);
}

/* Returns true if this timer has been set at least once, ready to time */
bool timer_was_initialized(timer_t* timer) {
  return (timer->interval != 0);
}

/* -----------------------------------------------------------------------------
 * API for stop watches.
 */

/* Clear a stop watch, making it ready to use */
void stopwatch_clear(stopwatch_t* stopwatch) {
    stopwatch->running       = false;
    stopwatch->residual_time = 0;
}

/* Start new timing using a stop watch. Accumulated time is cleared */
void stopwatch_start(stopwatch_t* stopwatch) {
    stopwatch->start_time    = xTaskGetTickCount();
    stopwatch->residual_time = 0;
    stopwatch->running       = true;
}

/* Continue a current timing using a stop watch. Accumulated time is not cleared */
void stopwatch_continue(stopwatch_t* stopwatch) {

  /* If we are already running, this is nothing to do. */
  if (stopwatch->running) {
    return;
  }

  stopwatch->start_time = xTaskGetTickCount();
  stopwatch->running    = true;
}

/* Stop a running stop watch */
void stopwatch_stop(stopwatch_t* stopwatch) {

    /* If we are running, get the elapsed time currently on the watch and add it
     to the residual time to keep it accumulated properly. */
    if (stopwatch->running) {
        stopwatch->residual_time += (xTaskGetTickCount() - stopwatch->start_time);
    }

    stopwatch->running = false;
}

/* Returns the total elapsed time on a stop watch, in milliseconds */
uint32_t stopwatch_elapsed_ms(stopwatch_t* stopwatch) {

  uint32_t elapsed;

    if (stopwatch->running) {
        elapsed = stopwatch->residual_time + (xTaskGetTickCount() - stopwatch->start_time);
    } else {
        elapsed = stopwatch->residual_time;
    }

  return elapsed;
}


/* -----------------------------------------------------------------------------
 * Initialize FreeRTOS and start the initial set of tasks.
 */
int main(void) {

    uint32_t loop;
    uint8_t pins1, pins2;

    /* Set the clocking to run at 80 MHz from the PLL */
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Create mutexes to guard the UART and SSI messaging.
    g_pUARTSemaphore = xSemaphoreCreateMutex();
    g_txMessageSemaphore = xSemaphoreCreateMutex();

    /* Initialize the UART and configure it for 115,200, 8-N-1 operation */
    //ConfigureUART();
    //v_printf("\n\nACS Node Box sensor module started.\n");

    // TBD - this code never sleeps so this call is moot
    //SysCtlPeripheralClockGating(true);


#ifdef SENSOR_CLOCK_DATA_TEST
    pins1 = GPIO_PIN_6 | GPIO_PIN_7;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, pins1);

    pins2 = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)) {}
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, pins2);

    while (1) {
        GPIOPinWrite(GPIO_PORTB_BASE, pins1, 0);
        GPIOPinWrite(GPIO_PORTG_BASE, pins2, 0);
        for (loop = 0; loop < 5000; loop++) {} // 750us
        GPIOPinWrite(GPIO_PORTB_BASE, pins1, pins1);
        GPIOPinWrite(GPIO_PORTG_BASE, pins2, pins2);
        for (loop = 0; loop < 5000; loop++) {} // 750us
    }
#endif

    /* Setup the interrupt service routines for the GPIO lines */
    GPIO_Setup_ISR();

    /* Create the sensor task */
    if (Sensor_Task_Init() != 0) {
        while (1) {}
    }

    /* Create the SSI communication task */
    if (SSI_Task_Init() != 0) {
        while (1) {}
    }


#ifdef ZERO
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0); // off

    // Enable the GPIO pin for the LED (PD0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3); // on
#endif


    /* All tasks are primed, enable interrupts */
    IntMasterEnable();

    /* Start the scheduler.  This should not return. */
    vTaskStartScheduler();

    /* In case the scheduler returns for some reason, print an error and loop forever */
    while(1) {
    }
}
