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

    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.

    while(1) {
    }
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
 * Initialize FreeRTOS and start the initial set of tasks.
 */
int main(void) {

    /* Set the clocking to run at 80 MHz from the PLL */
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Create mutexes to guard the UART and SSI messaging.
    g_pUARTSemaphore = xSemaphoreCreateMutex();
    g_txMessageSemaphore = xSemaphoreCreateMutex();

    /* Initialize the UART and configure it for 115,200, 8-N-1 operation */
    ConfigureUART();
    v_printf("\n\nACS Node Box sensor module started.\n");

    // TBD - this code never sleeps so this call is moot
    //SysCtlPeripheralClockGating(true);

    /* Init all the GPIO peripherals here so the tasks don't have to duplicate this code */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)) {}

    // Create the sensor task.
    if(Sensor_Task_Init() != 0) {
        while(1) {
        }
    }

    // Create the SSI communication task
    if(SSI_Task_Init() != 0) {
        while(1) {
        }
    }


#ifdef ZERO

    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0); // off



    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}

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
