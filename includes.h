/*
 * includes.h - Common #includes for the ACS Node Box Sensor firmware project
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_

// Feature switches
#undef VIRTUAL_UART_SUPPORT   // define this to 1 if supported


#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

// Board support includes
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_i2c.h>
#include <inc/hw_ssi.h>

#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <utils/uartstdio.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "priorities.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Tasks
#include "main.h"
#include "spi_task.h"
#include "sensor_task.h"

// External interfaces
#include "SSI0.h"


#endif /* INCLUDES_H_ */




