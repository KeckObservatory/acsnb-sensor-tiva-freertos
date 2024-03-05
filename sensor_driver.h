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

#include "includes.h"

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_DRIVER_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

/* Name the sensors.
 * Note that sensors are named 1 to 6 and are indexed from 0.
 */
typedef enum {
    SENSOR1                   = 0,
    SENSOR2,
    SENSOR3,
    SENSOR4,
    SENSOR5,
    SENSOR6,
    MAX_SENSORS // 6
} sensor_name_t;

/* The flags used to indicate a sensor conversion is complete and ready for
 * retrieval */
EXTERN bool sensor1_ready;
EXTERN bool sensor2_ready;
EXTERN bool sensor3_ready;
EXTERN bool sensor4_ready;
EXTERN bool sensor5_ready;
EXTERN bool sensor6_ready;

/* Define the function prototypes early, as they are used in a struct below */
EXTERN void Sensor1Ready(void);
EXTERN void Sensor2Ready(void);
EXTERN void Sensor3Ready(void);
EXTERN void Sensor4Ready(void);
EXTERN void Sensor5Ready(void);
EXTERN void Sensor6Ready(void);

/* -----------------------------------------------------------------------------
 * Sensor I2C bus control
 * -----------------------------------------------------------------------------
 */

/* Define the ISR function call to make a function pointer out of it */
typedef void (*isrFunc)(void);

/*
 * Group all the Tiva API constants into a structure.  This allows use of a single
 * function to work with any of the 6 I2C busses, without duplication of code.
 */
typedef struct {
    uint32_t peripheral;   /* The I2C bus peripheral */
    uint32_t periph_base;  /* The I2C controller base address */
    uint32_t port_base;    /* The GPIO port for the I2C pins */
    uint32_t scl;          /* I2C clock pin alias */
    uint32_t scl_pin;      /* I2C clock pin number */
    uint32_t sda;          /* I2C data pin alias */
    uint32_t sda_pin;      /* I2C data pin alias */
    uint32_t rdy_port;     /* The GPIO port for the ready signal pin */
    uint32_t rdy_pin;      /* Ready signal pin alias */
    isrFunc isr;           /* Ready signal interrupt service routine */
    bool *isr_flag;        /* The flag to set when the ready signal is asserted */
} sensor_io_t;

/* The following port and pin assignments are found on sheet 6 of the Kona
 * node box schematic, titled "SENSOR I2C INTERFACE".
 *
 * Define this structure only for use with sensor_driver.c, because it is
 * initialized here.  The EXTERN mechanism does not work for initialized
 * values.
 */
#ifdef SENSOR_DRIVER_C_
sensor_io_t sensor_io[MAX_SENSORS] = {
        [SENSOR1].peripheral  = SYSCTL_PERIPH_I2C0,
        [SENSOR1].periph_base = I2C0_BASE,
        [SENSOR1].port_base   = GPIO_PORTB_BASE,
        [SENSOR1].scl         = GPIO_PB2_I2C0SCL,
        [SENSOR1].scl_pin     = GPIO_PIN_2,
        [SENSOR1].sda         = GPIO_PB3_I2C0SDA,
        [SENSOR1].sda_pin     = GPIO_PIN_3,
        [SENSOR1].rdy_port    = GPIO_PORTA_BASE,
        [SENSOR1].rdy_pin     = GPIO_PIN_7,
        [SENSOR1].isr         = Sensor1Ready,
        [SENSOR1].isr_flag    = &sensor1_ready,

        [SENSOR2].peripheral  = SYSCTL_PERIPH_I2C1,
        [SENSOR2].periph_base = I2C1_BASE,
        [SENSOR2].port_base   = GPIO_PORTG_BASE,
        [SENSOR2].scl         = GPIO_PG4_I2C1SCL,
        [SENSOR2].scl_pin     = GPIO_PIN_4,
        [SENSOR2].sda         = GPIO_PG5_I2C1SDA,
        [SENSOR2].sda_pin     = GPIO_PIN_5,
        [SENSOR2].rdy_port    = GPIO_PORTB_BASE,
        [SENSOR2].rdy_pin     = GPIO_PIN_5,
        [SENSOR2].isr         = Sensor2Ready,
        [SENSOR2].isr_flag    = &sensor2_ready,

        [SENSOR3].peripheral  = SYSCTL_PERIPH_I2C2,
        [SENSOR3].periph_base = I2C2_BASE,
        [SENSOR3].port_base   = GPIO_PORTE_BASE,
        [SENSOR3].scl         = GPIO_PE4_I2C2SCL,
        [SENSOR3].scl_pin     = GPIO_PIN_4,
        [SENSOR3].sda         = GPIO_PE5_I2C2SDA,
        [SENSOR3].sda_pin     = GPIO_PIN_5,
        [SENSOR3].rdy_port    = GPIO_PORTC_BASE,
        [SENSOR3].rdy_pin     = GPIO_PIN_4,
        [SENSOR3].isr         = Sensor3Ready,
        [SENSOR3].isr_flag    = &sensor3_ready,

        [SENSOR4].peripheral  = SYSCTL_PERIPH_I2C3,
        [SENSOR4].periph_base = I2C3_BASE,
        [SENSOR4].port_base   = GPIO_PORTG_BASE,
        [SENSOR4].scl         = GPIO_PG0_I2C3SCL,
        [SENSOR4].scl_pin     = GPIO_PIN_0,
        [SENSOR4].sda         = GPIO_PG1_I2C3SDA,
        [SENSOR4].sda_pin     = GPIO_PIN_1,
        [SENSOR4].rdy_port    = GPIO_PORTD_BASE,
        [SENSOR4].rdy_pin     = GPIO_PIN_7,
        [SENSOR4].isr         = Sensor4Ready,
        [SENSOR4].isr_flag    = &sensor4_ready,

        [SENSOR5].peripheral  = SYSCTL_PERIPH_I2C4,
        [SENSOR5].periph_base = I2C4_BASE,
        [SENSOR5].port_base   = GPIO_PORTG_BASE,
        [SENSOR5].scl         = GPIO_PG2_I2C4SCL,
        [SENSOR5].scl_pin     = GPIO_PIN_2,
        [SENSOR5].sda         = GPIO_PG3_I2C4SDA,
        [SENSOR5].sda_pin     = GPIO_PIN_3,
        [SENSOR5].rdy_port    = GPIO_PORTE_BASE,
        [SENSOR5].rdy_pin     = GPIO_PIN_0,
        [SENSOR5].isr         = Sensor5Ready,
        [SENSOR5].isr_flag    = &sensor5_ready,

        [SENSOR6].peripheral  = SYSCTL_PERIPH_I2C5,
        [SENSOR6].periph_base = I2C5_BASE,
        [SENSOR6].port_base   = GPIO_PORTB_BASE,
        [SENSOR6].scl         = GPIO_PB6_I2C5SCL,
        [SENSOR6].scl_pin     = GPIO_PIN_6,
        [SENSOR6].sda         = GPIO_PB7_I2C5SDA,
        [SENSOR6].sda_pin     = GPIO_PIN_7,
        [SENSOR6].rdy_port    = GPIO_PORTF_BASE,
        [SENSOR6].rdy_pin     = GPIO_PIN_4,
        [SENSOR6].isr         = Sensor6Ready,
        [SENSOR6].isr_flag    = &sensor6_ready,
};
#else
/* extern the definition so other C files can use it */
extern sensor_io_t sensor_io[];
#endif

/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN bool I2CMasterTimeout(uint32_t ui32Base);
EXTERN void I2CInit(sensor_name_t sensor);
EXTERN int8_t I2CSend(uint32_t base, uint32_t slave_addr, uint8_t *buf, uint8_t len);
EXTERN int8_t I2CReceive(uint32_t base, uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);
EXTERN void SensorReadySet(sensor_name_t sensor, bool enable);

#undef EXTERN

#endif /* SENSOR_DRIVER_H_ */
