/*
 * uDMA.c
 */

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_ssi.h>
#include <inc/hw_memmap.h>
#include <driverlib/udma.h>
#include <driverlib/sysctl.h>
#include "uDMA.h"

#ifdef ZERO

/*
 * The control table used by the uDMA controller.  This table must be aligned
 * to a 1024 byte boundary.
 */
#pragma DATA_ALIGN(DMAcontroltable, 1024)
uint8_t DMAcontroltable[1024];

/*
 * Initialization of uDMA peripheral.
 */
void uDMAInit(void) {

    /*
     * Starting DMA Configuration.
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)) {}
    uDMAEnable();

    /*
     * Point at the control table to use for channel
     * control structures.
     */
    uDMAControlBaseSet(&DMAcontroltable[0]);
}

void uDMAInitSSI0(void) {

    /*
     * Assign the uDMA channel to the peripherals and
     * disable all the attributes in case any was set.
     */
    uDMAChannelAssign(UDMA_CH10_SSI0RX);
    uDMAChannelAssign(UDMA_CH11_SSI0TX);
    uDMAChannelAttributeDisable(UDMA_CH10_SSI0RX, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);
    uDMAChannelAttributeDisable(UDMA_CH11_SSI0TX, UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    /*
     * Set the channel control.
     * In this case we are working with string of char values,
     * so we use UDMA_SIZE_8. No increment is used on source,
     * because it will be pointing to SSI_O_DR register, and
     * increment of 8 bits are used because we will fill a
     * string of char values "bufferRx".
     */
    uDMAChannelControlSet(UDMA_CH10_SSI0RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_DST_INC_8 | UDMA_SRC_INC_NONE | UDMA_ARB_8);
    uDMAChannelControlSet(UDMA_CH11_SSI0TX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_8);
}

/*
 * Prepares the receiving for next data.
 *
 * \param RxMessageBuffer is the address of the buffer where the
 * data will be stored.
 * \param dataLength is the length of the message to be received.
 *
 * It prepares the receiving of data storing on RxMessageBuffer
 * a number of bytes defined by dataLength.
 * Use "sizeof()" function to facilitate length count.
 */
void uDMAReceiveSSI0(char RxMessageBuffer[], uint16_t dataLength) {

    uDMAChannelAttributeEnable(UDMA_CH10_SSI0RX, UDMA_ATTR_USEBURST);
    uDMAChannelTransferSet(UDMA_CH10_SSI0RX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, (void *)(SSI0_BASE + SSI_O_DR), &RxMessageBuffer[0], dataLength);
    uDMAChannelEnable(UDMA_CH10_SSI0RX);
    uDMAChannelRequest(UDMA_CH10_SSI0RX);
}

/*
 * Send a string vector of the argument.
 *
 * \param TxMessageBuffer is the address of the message string.
 * \param dataLength is the length of the message to be sent.
 *
 * It receives the message string address and send it through
 * uDMA.
 * Use "sizeof()" function to facilitate length count.
 */
void uDMASendSSI0(char TxMessageBuffer[], uint16_t dataLength) {

    uDMAChannelAttributeEnable(UDMA_CH11_SSI0TX, UDMA_ATTR_USEBURST);
    uDMAChannelTransferSet(UDMA_CH11_SSI0TX | UDMA_PRI_SELECT, UDMA_MODE_BASIC, &TxMessageBuffer[0], (void *)(SSI0_BASE + SSI_O_DR), dataLength);
    uDMAChannelEnable(UDMA_CH11_SSI0TX);
    uDMAChannelRequest(UDMA_CH11_SSI0TX);
}

#endif


/*
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


