/*
 * gpio_isr.h - Interrupt service routines for GPIO lines.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef ISR_H_
#define ISR_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef ISR_H_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN void GPIOSetupISR(void);
EXTERN void GPIOPortAIntHandler(void);
EXTERN void GPIOPortBIntHandler(void);
EXTERN void GPIOPortCIntHandler(void);
EXTERN void GPIOPortDIntHandler(void);
EXTERN void GPIOPortEIntHandler(void);
EXTERN void GPIOPortFIntHandler(void);

#undef EXTERN

#endif /* ISR_H_ */

