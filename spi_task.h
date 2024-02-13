//*****************************************************************************
//
// spi_task.h - Prototypes for the SPI interface task.
//
//*****************************************************************************

#ifndef __SPI_TASK_H__
#define __SPI_TASK_H__

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"

#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern void SPI_ISR(void);
extern uint32_t SPI_Task_Init(void);


#endif // __SPI_TASK_H__
