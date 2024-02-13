/*
 * SSI0.c
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <utils/uartstdio.h>

#include "SSI0.h"
#include "uDMA.h"

/* DMA Channel Interrupt Status (DMACHIS) flags */
#define UDMA_SSI0_RX       0b0000010000000000  // Channel 10
#define UDMA_SSI0_TX       0b0000100000000000  // Channel 11


/*
 * Variables and pointers used.
 */
static uint8_t *SSI0_RxPointer;
static uint8_t *SSI0_TxPointer;
static bool *dataReceived;
static bool *dataSent;
static uint16_t dataLength;

#ifdef ZERO
/*
 * Initialization of communication by SSI/uDMA.
 *
 * \param received and sent are the address of the respective address for flags.
 * \param RxBuffer is the address to the buffer that will receive data.
 *
 * Initializes SSI0 communication and receives pointers for
 * flags and buffer for communication purposes.
 */
void SSI0Init_old(bool *received, bool *sent, uint8_t RxBuffer[], uint8_t TxBuffer[], uint16_t length) {

    /*
     * Initialize flags for receiving message.
     */
    SSI0_RxPointer = &RxBuffer[0];
    SSI0_TxPointer = &TxBuffer[0];

    dataReceived = received;
    *dataReceived = false;
    dataSent = sent;
    *dataSent = false;

    dataLength = length;

    /*
     * Configure SSI0 pins (PA0/Rx and PA1/Tx) on TIVA.
     * Configure it to operate even if the CPU is in sleep.
     */

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    /* Wait for the SSI0 module to be ready. */
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)) {};

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI0);

    /* Enable the SSI module. */
    SSIEnable(SSI0_BASE);

    /* Set the pins for SSI usage */
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* Configure the peripheral for slave mode */
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);

    /* Register an interrupt handler for FIFO events */
    ////SSIIntRegister(SSI0_BASE, SSI0Interrupt);

    /* Interrupt when the transmit FIFO is half full or less */
    ////SSIIntEnable(SSI0_BASE, SSI_TXFF);

    /* Configure the uDMA operation on SSI0. */
    uDMAInit();
    uDMAInitSSI0();

    //uDMAIntRegister(INT_UDMA, SSI0Interrupt);
    /////uDMAIntRegister(UDMA_SSI0_RX, SSI0Interrupt);
    /////uDMAIntRegister(UDMA_SSI0_TX, SSI0Interrupt);

    SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);


    /*
     * Enable the uDMA interface for both TX and RX channels
     * and enable the uDMA/Rx channel for receiving.
     * uDMA/Tx is not enabled here (uDMASendSSI0) yet
     * because there is no data to send and may cause
     * undesirable interruption.
     */
    uDMAReceiveSSI0((char *) SSI0_RxPointer, dataLength);
    uDMASendSSI0((char *) SSI0_TxPointer, dataLength);

    IntEnable(INT_SSI0);
}
#endif

void SSI0Init(bool *received, bool *sent, uint8_t RxBuffer[], uint8_t TxBuffer[], uint16_t length) {

    /*
     * Initialize flags for receiving message.
     */
    SSI0_RxPointer = &RxBuffer[0];
    SSI0_TxPointer = &TxBuffer[0];

    dataReceived = received;
    *dataReceived = false;
    dataSent = sent;
    *dataSent = false;

    dataLength = length;

    uint32_t trashBin[1] = {0};

    //
    // Enable the SSI0 Peripheral.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure GPIO Pins for SSI0 mode.
    //
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_SLAVE, 5000000, 8);
    //SSIConfigSetExpClk(SSI0_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI0_BASE);

    /* Clear SSI0 RX Buffer */
    while (SSIDataGetNonBlocking(SSI0_BASE, &trashBin[0])) {}
}


/*
 * Interrupt handler for uDMA/SSI.
 */
void SSI0IntHandler(void) {
    uint32_t ui32Status;
    uint32_t ui32Mode;

    ui32Status = SSIIntStatus(SSI0_BASE, 1);

    SSIIntClear(SSI0_BASE, ui32Status);

    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT);

    if(ui32Mode == UDMA_MODE_STOP)
    {
        //g_ui32SSIRxCount++;

        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   SSI0_RxPointer,
                                   dataLength);

        uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);
    }

    if(!uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX))
    {
        //g_ui32SSITxCount++;

        uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC,
                                   SSI0_TxPointer,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   dataLength);

        uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
    }
}


#ifdef ZERO
void SSI0Interrupt(void) {

    uint32_t ssi_interrupt_status_raw, ssi_interrupt_status_masked;
    uint32_t udma_interrupt_status;

    bool tx_fifo_half_empty, rx_fifo_half_full, rx_timeout, rx_overrun;
    bool udma_tx, udma_rx;

    /* Determine which interrupt got us here */
    ////ssi_interrupt_status_raw = SSIIntStatus(SSI0_BASE, false);
    ////ssi_interrupt_status_masked = SSIIntStatus(SSI0_BASE, true);
    udma_interrupt_status = uDMAIntStatus();

    /* Clear the SPI interrupt now because it takes time to clear, can't do it at the end */
    ////SSIIntClear(SSI0_BASE, SSI_TXFF);

    /* Get the SSI status register */
    //ssisr = HWREG(SSI0_BASE + SSI_O_SR);

    /* Decode the flags of the status register */
    tx_fifo_half_empty = ssi_interrupt_status_masked & SSI_TXFF;  // TX FIFO half full or less
    rx_fifo_half_full = ssi_interrupt_status_masked & SSI_RXFF;  // RX FIFO half full or more
    rx_timeout = ssi_interrupt_status_masked & SSI_RXTO;  // RX timeout
    rx_overrun = ssi_interrupt_status_masked & SSI_RXOR;  // RX overrun

    udma_rx = udma_interrupt_status & UDMA_SSI0_RX;
    udma_tx = udma_interrupt_status & UDMA_SSI0_TX;


/*
#define SSI_TXEOT               0x00000040  // Transmit FIFO is empty
#define SSI_DMATX               0x00000020  // DMA Transmit complete
#define SSI_DMARX               0x00000010  // DMA Receive complete
#define SSI_TXFF                0x00000008  // TX FIFO half full or less
#define SSI_RXFF                0x00000004  // RX FIFO half full or more
#define SSI_RXTO                0x00000002  // RX timeout
#define SSI_RXOR                0x00000001  // RX overrun
*/

    /*
     * uDMA receive interrupt
     */
    if (udma_rx) {

        *dataReceived = true;
        uDMAReceiveSSI0((char *) SSI0_RxPointer, dataLength);
        uDMASendSSI0((char *) SSI0_TxPointer, dataLength);

        uDMAIntClear(UDMA_SSI0_RX);
    }

    /*
     * uDMA transmit interrupt
     */
    if (udma_tx) {


        uDMAIntClear(UDMA_SSI0_TX);
    }

    /*
     * SSI
     */
    if (tx_fifo_half_empty) {

        //SSI0_CheckAndSend();
        //uDMAIntClear(UDMA_SSI0_TX);
        SSIIntClear(SSI0_BASE, SSI_TXFF);
    }



    /*
     * Clear any other interrupt on SSI0 Interrupt Register.
     */
    //UARTIntClear(SSI0_BASE, SSI0_Int.RAW);
    //UARTIntClear(SSI0_BASE, SSI0_Int.MASKED);
}

/*
 * Send the chosen message.
 *
 * /param TxBuffer[] is the string or concatenated message.
 */
void SSI0SendMessage(void) {
    uDMASendSSI0((char *) SSI0_TxPointer, dataLength);
}
#endif


void InitSPITransfer(void) {

    //uint_fast16_t ui16Idx;

    //
    // Enable the uDMA interface for both TX and RX channels.
    //
    SSIDMAEnable(SSI0_BASE, SSI_DMA_RX | SSI_DMA_TX);

    //****************************************************************************
    //uDMA SSI0 RX
    //****************************************************************************

    //
    // Put the attributes in a known state for the uDMA SSI0RX channel.  These
    // should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0RX,
                                UDMA_ATTR_USEBURST | UDMA_ATTR_ALTSELECT |
                                (UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK));

    //
    // Configure the control parameters for the primary control structure for
    // the SSIORX channel.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                          UDMA_ARB_4);

    //
    // Set up the transfer parameters for the SSI0RX Channel
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0RX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           (void *)(SSI0_BASE + SSI_O_DR),
                           SSI0_RxPointer,
                           dataLength);


    //****************************************************************************
    //uDMA SSI0 TX
    //****************************************************************************

    //
    // Put the attributes in a known state for the uDMA SSI0TX channel.  These
    // should already be disabled by default.
    //
    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX,
                                UDMA_ATTR_ALTSELECT |
                                UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    //
    // Set the USEBURST attribute for the uDMA SSI0TX channel.  This will
    // force the controller to always use a burst when transferring data from
    // the TX buffer to the SSI0.  This is somewhat more effecient bus usage
    // than the default which allows single or burst transfers.
    //
    uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);

    //
    // Configure the control parameters for the SSI0 TX.
    //
    uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
                          UDMA_ARB_4);


    //
    // Set up the transfer parameters for the uDMA SSI0 TX channel.  This will
    // configure the transfer source and destination and the transfer size.
    // Basic mode is used because the peripheral is making the uDMA transfer
    // request.  The source is the TX buffer and the destination is theUART0
    // data register.
    //
    uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           SSI0_TxPointer,
                           (void *)(SSI0_BASE + SSI_O_DR),
                           dataLength);

    //
    // Now both the uDMA SSI0 TX and RX channels are primed to start a
    // transfer.  As soon as the channels are enabled, the peripheral will
    // issue a transfer request and the data transfers will begin.
    //
    uDMAChannelEnable(UDMA_CHANNEL_SSI0RX);
    uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

    //
    // Enable the SSI0 DMA TX/RX interrupts.
    //
    SSIIntEnable(SSI0_BASE, SSI_DMATX | SSI_DMARX);

    //
    // Enable the SSI0 peripheral interrupts.
    //
    IntEnable(INT_SSI0);

}
