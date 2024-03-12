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

/* The mutex that protects concurrent access of UART from multiple tasks */
EXTERN xSemaphoreHandle g_pUARTSemaphore;

/* The mutex that protects concurrent access of SPI messaging buffers */
EXTERN xSemaphoreHandle g_txMessageSemaphore;



/* This defines a timer for general use. It works off the RTOS tick/clock at 1 millisecond precision. */
typedef struct {
  uint32_t start_time;
  uint32_t interval;
} timer_t;

/* This defines a stopwatch for general use. It works off the RTOS tick/clock at 1 millisecond precision. */
typedef struct {
  bool running;
  uint32_t start_time;
  uint32_t residual_time;      /* Accumulated time on the stopwatch prior to the current timing cycle. */
} stopwatch_t;


/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN void v_printf(const char *pcString, ...);
EXTERN void timer_set(timer_t* timer, uint32_t interval);
EXTERN void timer_start(timer_t* timer);
EXTERN bool timer_expired(timer_t* timer);
EXTERN void timer_continue(timer_t* timer);
EXTERN void timer_expire(timer_t* timer);
EXTERN uint32_t timer_time_remaining(timer_t* timer);
EXTERN uint32_t timer_elapsed_time(timer_t* timer);
EXTERN bool timer_was_initialized(timer_t* timer);
EXTERN void stopwatch_clear(stopwatch_t* stopwatch);
EXTERN void stopwatch_start(stopwatch_t* stopwatch);
EXTERN void stopwatch_continue(stopwatch_t* stopwatch);
EXTERN void stopwatch_stop(stopwatch_t* stopwatch);
EXTERN uint32_t stopwatch_elapsed_ms(stopwatch_t* stopwatch);

#undef EXTERN

#endif /* MAIN_H_ */
