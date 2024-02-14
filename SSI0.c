/*
 * SSI0.c
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
#include <stdio.h>

#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ssi.h>

#include <driverlib/ssi.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/udma.h>

#include "SSI0.h"


/* Variables and pointers used */
static uint8_t *SSI0_RxPointer;
static uint8_t *SSI0_TxPointer;
static bool *dataReceived;
static bool *dataSent;
static uint16_t dataLength;

void SSI0Init(bool *received, bool *sent, uint8_t RxBuffer[], uint8_t TxBuffer[], uint16_t length) {

    uint32_t trashBin[1] = {0};

    /* Initialize flags for receiving message */
    SSI0_RxPointer = &RxBuffer[0];
    SSI0_TxPointer = &TxBuffer[0];

    dataReceived = received;
    *dataReceived = false;
    dataSent = sent;
    *dataSent = false;
    dataLength = length;

    /* Enable the SSI0 Peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI0);

    /* Configure GPIO Pins for SSI0 mode */
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure SSI clock to run at 5MHz */
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);

    SSIEnable(SSI0_BASE);

    /* Clear SSI0 RX Buffer */
    while (SSIDataGetNonBlocking(SSI0_BASE, &trashBin[0])) {}
}


/* Interrupt handler for uDMA/SSI */
void SSI0IntHandler(void) {

    uint32_t ui32Status;
    uint32_t ui32Mode;

    ui32Status = SSIIntStatus(SSI0_BASE, 1);

    SSIIntClear(SSI0_BASE, ui32Status);

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
}


void InitSPITransfer(void) {

    /* Enable the uDMA interface for both TX and RX channels */
    SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);

    //****************************************************************************
    // uDMA SSI0 RX
    //****************************************************************************

    /* Put the attributes in a known state for the uDMA SSI0RX channel.  These
       should already be disabled by default. */
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0RX,
                                UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                (UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK));

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

/* The interrupt handler for uDMA errors. This interrupt will occur if the
   uDMA encounters a bus error while trying to perform a transfer. This
   handler just increments a counter if an error occurs. */
void uDMAErrorHandler(void) {

    if (uDMAErrorStatusGet()) {

        uDMAErrorStatusClear();
        /* TODO: This will be properly implemented. */
        /* perError.uDMA++; */
    }
}

