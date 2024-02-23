/*
 * sensor_driver.h - Prototypes for the I2C interfaces.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SENSOR_DRIVER_H_
#define SENSOR_DRIVER_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_DRIVER_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"


/* -----------------------------------------------------------------------------
 * Function prototypes
 */



#undef EXTERN

#endif /* SENSOR_DRIVER_H_ */
