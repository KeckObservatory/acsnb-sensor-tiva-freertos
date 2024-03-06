/*
 * gpio_isr.c - Interrupt service routines for GPIO lines.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 * This file contains the interrupt service routines for all GPIO based interrupts
 * in the sensor code.
 *
 * Sensor 1 ready on PORTA pin 7
 * SSI0 chip select on PORTA pin 3
 *
 * Sensor 2 ready on PORTB pin 5
 * Sensor 3 ready on PORTC pin 4
 * Sensor 4 ready on PORTD pin 7
 * Sensor 5 ready on PORTE pin 0
 * Sensor 6 ready on PORTF pin 4
 */

#define ISR_C_
#include "includes.h"

/* -----------------------------------------------------------------------------
 * Connect the individual GPIO lines to their ISRs.
 */
void GPIOSetupISR(void) {

    /* Clear the ready flags */
    sensor1_ready = false;
    sensor2_ready = false;
    sensor3_ready = false;
    sensor4_ready = false;
    sensor5_ready = false;
    sensor6_ready = false;

    /* Setup the Ready lines for the sensors */
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_INT_PIN_3 | GPIO_INT_PIN_7);  /* Also setup SSI0 chip select */
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_INT_PIN_7);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_INT_PIN_4);

    /* Tie the interrupt handlers */
    GPIOIntRegister(GPIO_PORTA_BASE, GPIOPortAIntHandler);
    GPIOIntRegister(GPIO_PORTB_BASE, GPIOPortBIntHandler);
    GPIOIntRegister(GPIO_PORTC_BASE, GPIOPortCIntHandler);
    GPIOIntRegister(GPIO_PORTD_BASE, GPIOPortDIntHandler);
    GPIOIntRegister(GPIO_PORTE_BASE, GPIOPortEIntHandler);
    GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortFIntHandler);

    /* From the AD7745 spec pg 7: A falling edge on this output indicates that a
     * conversion on enabled channel(s) has been finished and the new data is available.
     * However, per node box schematic, the signal is inverted  - so use the rising edge!
     */
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_3, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_INT_PIN_5, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_INT_PIN_4, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_INT_PIN_7, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_INT_PIN_0, GPIO_RISING_EDGE);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_INT_PIN_4, GPIO_RISING_EDGE);

    /* For SSI0, interrupt on the rising edge of the chip select, which is the signal that
     * the master has ended the transaction. */
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_INT_PIN_7, GPIO_RISING_EDGE);

    /* Clear any interrupts before we enable the ISRs */
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3 | GPIO_INT_PIN_7);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_7);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_4);

    /* Enable the interrupts */
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_3 | GPIO_INT_PIN_7);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_7);
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_0);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 1 ready on PORTA pin 7
 * SSI0 chip select on PORTA pin 3
 */
void GPIOPortAIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTA_BASE, true);

    /* Sensor 1 ready line is on pin 7 */
    if (int_status & GPIO_INT_PIN_7) {

        /* Set the flag that the conversion is complete */
        sensor1_ready = true;
    }

    /* SSI0 chip select is on pin 3 */
    if (int_status & GPIO_INT_PIN_3) {

        /* There might be bytes left over in the transmit FIFO, we cannot let them be sent!
         * Therefore we must reset the SSI0 device */
        SSIDisable(SSI0_BASE);
        SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);
        SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);
        SSIEnable(SSI0_BASE);

        /* Copy the outbound message to the buffer the DMA will read from */
        memcpy(SSI0_tx_dma_pointer, SSI0_tx_pointer, SSI_MESSAGE_LENGTH);

        /* SSI0 is fully reset.  Drive a new DMA transfer of the buffer */
        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   SSI0_rx_pointer,
                                   SSI0_data_length);
        uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);

        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   SSI0_tx_dma_pointer,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   SSI0_data_length);
        uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

        /* Enable the DMA transfer */
        SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);

        /* Set the flag that the message has been received */
        *SSI0_msg_ready = true;
    }

    /* Clear the interrupt */
    GPIOIntClear(GPIO_PORTA_BASE, int_status);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 2 ready on PORTB pin 5
 */
void GPIOPortBIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTB_BASE, true);

    if (int_status & GPIO_INT_PIN_5) {
        sensor2_ready = true;
    }

    GPIOIntClear(GPIO_PORTB_BASE, int_status);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 3 ready on PORTC pin 4
 */
void GPIOPortCIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTC_BASE, true);

    if (int_status & GPIO_INT_PIN_4) {
        sensor3_ready = true;
    }

    GPIOIntClear(GPIO_PORTC_BASE, int_status);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 4 ready on PORTD pin 7
 */
void GPIOPortDIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTD_BASE, true);

    if (int_status & GPIO_INT_PIN_7) {
        sensor4_ready = true;
    }

    GPIOIntClear(GPIO_PORTD_BASE, int_status);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 5 ready on PORTE pin 0
 */
void GPIOPortEIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTE_BASE, true);

    if (int_status & GPIO_INT_PIN_0) {
        sensor5_ready = true;
    }

    GPIOIntClear(GPIO_PORTE_BASE, int_status);
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for PORTA GPIO lines.
 * Sensor 6 ready on PORTF pin 4
 */
void GPIOPortFIntHandler(void) {

    uint32_t int_status = GPIOIntStatus(GPIO_PORTF_BASE, true);

    if (int_status & GPIO_INT_PIN_4) {
        sensor6_ready = true;
    }

    GPIOIntClear(GPIO_PORTF_BASE, int_status);
}




