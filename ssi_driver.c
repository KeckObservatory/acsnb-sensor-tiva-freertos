/*
 * ssi_driver.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define SSI_DRIVER_C_
#include "includes.h"

/* Variables and pointers used */
static uint8_t *SSI0_RxPointer;
static uint8_t *SSI0_TxPointer;
static bool *SSI0_msg_ready;

static uint16_t dataLength;

/* -----------------------------------------------------------------------------
 * Initialize the SSI0 device.
 */
void SSI0Init(uint8_t rx_buffer[], uint8_t tx_buffer[], uint16_t length, bool *msg_ready) {

    uint32_t trashBin[1] = {0};

    /* Initialize flags for receiving message */
    SSI0_RxPointer = &rx_buffer[0];
    SSI0_TxPointer = &tx_buffer[0];
    SSI0_msg_ready = msg_ready;

    dataLength = length;

    /* Enable the uDMA and SSI0 peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI0);

    /* Configure GPIO Pins for SSI0 slave mode */
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure SSI clock to run at 5MHz */
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);

    /* Connect an interrupt handler to the chip select pin.  This will be used to drive
     * the next transaction. */
    GPIOIntRegister(GPIO_PORTA_BASE, SSI0SlaveSelectIntHandler);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3);

    /* Turn on the SSI0 */
    SSIEnable(SSI0_BASE);

    /* Clear SSI0 RX Buffer */
    while (SSIDataGetNonBlocking(SSI0_BASE, &trashBin[0])) {}
}

/* -----------------------------------------------------------------------------
 * Interrupt handler for the SSI0 chip select rising edge (when it's de-asserted)
 */
void SSI0SlaveSelectIntHandler(void) {

    /* Clear the interrupt that signaled the end of chip select to this device */
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);

    /* There might be bytes left over in the transmit FIFO, we cannot let them be sent!
     * Therefore we must reset the SSI0 device */
    SSIDisable(SSI0_BASE);
    SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);
    SSIEnable(SSI0_BASE);

    /* Copy the outbound message to the buffer the DMA will read from */
    memcpy(SSI0_TxPointer, tx_message_out.buf, SSI_MESSAGE_LENGTH);

    /* SSI0 is fully reset.  Drive a new DMA transfer of the buffer */
    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               (void *)(SSI0_BASE + SSI_O_DR),
                               SSI0_RxPointer,
                               dataLength);
    uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);

    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               SSI0_TxPointer,
                               (void *)(SSI0_BASE + SSI_O_DR),
                               dataLength);
    uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

    /* Enable the DMA transfer */
    SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);

    /* Set the flag that the message has been received */
    *SSI0_msg_ready = true;
}


/* -----------------------------------------------------------------------------
 * Interrupt handler for uDMA/SSI
 */
void SSI0IntHandler(void) {

    uint32_t ui32Status;
    //uint32_t ui32Mode;

    /* Get the masked interrupt status */
    ui32Status = SSIIntStatus(SSI0_BASE, 1);

    /* Clear all the interrupts */
    SSIIntClear(SSI0_BASE, ui32Status);

    /* Block below is commented out for now.  This part is done in the
     * interrupt handler for the chip select.  Regardless of whether the DMA
     * is done or not, we are going to re-start it, over in that routine! */
#ifdef ZERO
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT);

    if (ui32Mode == UDMA_MODE_STOP) {

        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   SSI0_RxPointer,
                                   dataLength);

        uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);
    }

    if (!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX)) {

        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   SSI0_TxPointer,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   dataLength);

        uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
    }
#endif

}


/* -----------------------------------------------------------------------------
 *
 */
void SSI0InitTransfer(void) {

    /* Enable the uDMA interface for both TX and RX channels */
    SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);

    //****************************************************************************
    // uDMA SSI0 RX
    //****************************************************************************

    /* Put the attributes in a known state for the uDMA SSI0RX channel.  These
       should already be disabled by default. */
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0RX,
            UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | (UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK));

    /* Configure the control parameters for the primary control structure for
       the SSIORX channel. */
    uDMAChannelControlSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                          UDMA_ARB_4);

    /* Set up the transfer parameters for the SSI0RX Channel */
    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           (void *)(SSI0_BASE + SSI_O_DR),
                           SSI0_RxPointer,
                           dataLength);

    //****************************************************************************
    // uDMA SSI0 TX
    //****************************************************************************

    /* Put the attributes in a known state for the uDMA SSI0TX channel.  These
       should already be disabled by default. */
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX,
            UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    /* Set the USEBURST attribute for the uDMA SSI0TX channel.  This will
       force the controller to always use a burst when transferring data from
       the TX buffer to the SSI0.  This is somewhat more efficient bus usage
       than the default which allows single or burst transfers. */
    uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);

    /* Configure the control parameters for the SSI0 TX */
    uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                          UDMA_ARB_4);

    /* Set up the transfer parameters for the uDMA SSI0 TX channel.  This will
       configure the transfer source and destination and the transfer size.
       Basic mode is used because the peripheral is making the uDMA transfer
       request.  The source is the TX buffer and the destination is theUART0
       data register. */
    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           SSI0_TxPointer,
                           (void *)(SSI0_BASE + SSI_O_DR),
                           dataLength);

    /* Now both the uDMA SSI0 TX and RX channels are primed to start a
       transfer.  As soon as the channels are enabled, the peripheral will
       issue a transfer request and the data transfers will begin. */
    uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);
    uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

    /* Enable the SSI0 DMA TX/RX interrupts */
    SSIIntEnable(SSI0_BASE, SSI_DMATX | SSI_DMARX);

    /* Enable the SSI0 peripheral interrupts */
    IntEnable(INT_SSI0);
}

/* -----------------------------------------------------------------------------
 * The interrupt handler for uDMA errors. This interrupt will occur if the
 * uDMA encounters a bus error while trying to perform a transfer. This
 * handler just increments a counter if an error occurs.
 */
void uDMAErrorHandler(void) {

    if (uDMAErrorStatusGet()) {

        uDMAErrorStatusClear();
        /* TODO: This will be properly implemented. */
        /* perError.uDMA++; */
    }
}

