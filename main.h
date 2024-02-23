/*
 * main.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "includes.h"

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef MAIN_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif


//*****************************************************************************
// The mutex that protects concurrent access of UART from multiple tasks.
//*****************************************************************************
EXTERN xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
// The mutex that protects concurrent access of SPI messaging buffers.
//*****************************************************************************
EXTERN xSemaphoreHandle g_txMessageSemaphore;

/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */
EXTERN void v_printf(const char *pcString, ...);

#undef EXTERN

#endif /* MAIN_H_ */
