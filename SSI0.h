/*
 * SSI0.h - Prototypes for the SSI0 interface.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SSI0_H_
#define SSI0_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_TASK_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN void SSI0SlaveSelectIntHandler(void);
EXTERN void SSI0Init(uint8_t RxBuffer[], uint8_t TxBuffer[], uint16_t length);
EXTERN void SSI0Interrupt(void);
EXTERN void SSI0SendMessage(void);
EXTERN void SSI0InitTransfer(void);
EXTERN void uDMAErrorHandler(void);

#undef EXTERN

#endif /* SSI0_H_ */
