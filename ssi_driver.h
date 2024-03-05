/*
 * ssi_driver.h - Prototypes for the SSI0 interface.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SSI_DRIVER_H_
#define SSI_DRIVER_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SSI_DRIVER_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN void SSI0SlaveSelectIntHandler(void);
EXTERN void SSI0Init(uint8_t rx_buffer[], uint8_t tx_buffer[], uint8_t tx_dma_buffer[], uint16_t length, bool *msg_ready);
EXTERN void SSI0Interrupt(void);
EXTERN void SSI0SendMessage(void);
EXTERN void SSI0InitTransfer(void);
EXTERN void uDMAErrorHandler(void);

#undef EXTERN

#endif /* SSI_DRIVER_H_ */
