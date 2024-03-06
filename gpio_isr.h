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
EXTERN void GPIO_Setup_ISR(void);
EXTERN void GPIO_PortA_Int_Handler(void);
EXTERN void GPIO_PortB_Int_Handler(void);
EXTERN void GPIO_PortC_Int_Handler(void);
EXTERN void GPIO_PortD_Int_Handler(void);
EXTERN void GPIO_PortE_Int_Handler(void);
EXTERN void GPIO_PortF_Int_Handler(void);

#undef EXTERN

#endif /* ISR_H_ */

