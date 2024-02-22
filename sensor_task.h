/*
 * sensor_task.h
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SENSOR_TASK_H_
#define SENSOR_TASK_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_TASK_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

#ifdef DEBUG_INTERRUPT
#define MAX_SENSOR_TIMEOUT_MS     5000
#define MAX_FAILED_INIT_WAIT_MS   5000
#else
// Run the sensor timeouts faster when debugging
#define MAX_SENSOR_TIMEOUT_MS     1000
#define MAX_FAILED_INIT_WAIT_MS   1000
#endif

#define MIN_TASK_SLEEP_MS         1
#define MIN_TEMP_READ_PERIOD_MS   1000
#define FILTER_COEFF              0.99333

// -----------------------------------------------------------------------------
// HDC1080 - Temperature and humidity sensor
#define HDC1080_ADDR              0x40
#define HDC1080_TMP_REG           0x00
#define HDC1080_HUM_REG           0x01

// HDC1080 configuration register definitions
// Bit 15: RST (1 = software reset)
// Bit 13: HEAT (0 = heater disabled)
// Bit 12: MODE (0 = temp OR humidity, 1 = temp AND humidity in sequence)
// Bit 11: BTST (0 = battery voltage > 2.8V)
// Bit 10: TRES (0 = 14 bit temperature measurement resolution)
// Bit 9+8: HRES (00 = 14 bit humidity  measurement resolution)
// All other bits reserved and must be 0
#define HDC1080_CFG_REG           0x02
#define HDC1080_CFG_MODE_T_OR_H   0b0000000000000000
#define HDC1080_CFG_MODE_T_AND_H  0b0001000000000000
#define HDC1080_TRIGGER_BOTH      0x00
#define HDC1080_TRIGGER_ONE       0x01
#define HDC1080_SB1               0xFB
#define HDC1080_SB2               0xFC
#define HDC1080_SB3               0xFD
#define HDC1080_MANUFID           0xFE
#define HDC1080_DEVICEID          0xFF

// -----------------------------------------------------------------------------
// AD7746 - Capacitance sensor
#define AD7746_ADDR               0x48
#define AD7746_WRITE              0x00
#define AD7746_READ               0x01

// Register definitions
#define AD7746_STATUS_REG         0x00
#define AD7746_CAP_SETUP_REG      0x07

// Voltage setup register definitions (spec pg 16)
// Bit 7: VTEN (1 = enables voltage/temperature channel for single conversion)
// Bit 6-5: VTMD1, VTMD0 Channel Input (00 = Internal temperature sensor)
// Bit 4: EXTREF (0 = select the on-chip internal reference)
// Bit 3-2: (00 - These bits must be 0 for proper operation)
// Bit 1: VTSHORT (0 = disable internal short of the voltage/temperature channel input for test purposes)
// Bit 0 VTCHOP (1 = sets internal chopping on the voltage/temperature channel / must be set to 1 for the specified voltage/temperature channel performance)

#define AD7746_VT_SETUP_REG       0x08
#define AD7746_VT_SETUP_DISABLE   0x00
#define AD7746_VT_SETUP_INT_TEMP  0b10000001

// Excitation setup register definitions (spec pg 17)
// Bit 7: CLKCTRL (0 = default, 1 = decrease frequencies by factor of 2)
// Bit 6: EXCON   (1 = excitation signal present on output during cap channel conversion AND during voltage/temp conversion)
// Bit 5: EXCB    (0 = disable EXCB pin as the excitation output)
// Bit 4: NOTEXCB (0 = disable EXCB pin as the inverted excitation output)
// Bit 3: EXCA    (1 = enable EXCA pin as the excitation output)
// Bit 2: NOTEXCA (0 = disable EXCA pin as the inverted excitation output)
// Bit 1,0: EXCLV1 EXCLV0 (excitation voltage level)
//          11 = Voltage on cap     = (+/- Vdd)/2
//               EXC Pin Low Level  = 0
//               EXC Pin High Level = Vdd

#define AD7746_EXC_SETUP_REG      0x09
#define AD7746_EXC_SET_A          0b01001011

#define AD7746_CFG_REG            0x0A
#define AD7746_CAP_OFFSET_H       0x0D
#define AD7746_CAP_OFFSET_L       0x0E
#define AD7746_CAP_GAIN_H         0x0F
#define AD7746_CAP_GAIN_L         0x10
#define AD7746_VOLT_GAIN_H        0x11
#define AD7746_VOLT_GAIN_L        0x12


/* Temperature conversion time selections (spec page 18) */
typedef enum {

  adtct20msSingle               = 0b00000010, // 20ms single conversion
  adtct32msSingle               = 0b01000010, // 32ms single conversion
  adtct62msSingle               = 0b10000010, // 62ms single conversion
  adtct122msSingle              = 0b11000010  // 122ms single conversion

} adTemperatureConversionTime;
#define DEFAULT_TEMPERATURE_CONVERSION_TIME adtct32msSingle
#define AD7746_CAP_VS_TEMP_TRIGGER_INTERVAL 10   // Trigger one temperature read every 10 cap reads


/* Conversion time selection choices (spec page 18) */
typedef enum {

  adct11msCont                  = 0x01, // 11ms continuous
  adct11msSingle                = 0x02, // 11ms single
  adct38msCont                  = 0x19, // 38.0ms continuous
  adct38msSingle                = 0x1A, // 38ms single
  adct109msSingle               = 0x3A  // 109.6ms single

} adConversionTime;

// Conversion time is selected via the SPI interface
#define FAST_CONVERSION_TIME      adct38msSingle
#define DEFAULT_CONVERSION_TIME   adct109msSingle



/* Single / differential capacitance selection choices */
typedef enum {
  //adcsC1D1                      = 0xA0, // CIN1, DIFF=1
  adcsC2D1                      = 0xE0, // CIN2, DIFF=1
  adcsC1D0                      = 0x80, // CIN1, DIFF=0
  adcsC2D0                      = 0xC0  // CIN2, DIFF=0

} adCapSelect;

#define DEFAULT_CAPACITOR_SELECT adcsC2D1



// -----------------------------------------------------------------------------
// PCA9536 - Relay driver to switch back to old ACS connection
#define PCA9536_ADDR              0x41
#define PCA9536_OUT_PORT_REG      0x01
#define PCA9536_OUT_PORT_RESET    0x00
#define PCA9536_OUT_PORT_NEW_ACS  0x05
#define PCA9536_OUT_PORT_OLD_ACS  0x0A
#define PCA9536_CONFIG_REG        0x03
#define PCA9536_CONFIG_ALL_OUTPUT 0x00

/* PCA9536 state, sets the old/new ACS relay positions */
typedef enum {

  swOldACS    = 0x00,
  swNewACS    = 0x01

} swRelayPositions;


// Task state machine discrete states
typedef enum {

  tsPOR                 = 0,
  tsInit                = 1,
  tsInitFailed          = 2,
  tsInitFailedWait      = 3,
  tsStart               = 4,
  tsRunning             = 5,
  tsRunFailed           = 6,
  tsRunFailedWait       = 7

} taskState;

// Task state data
typedef struct {

  uint32_t         i2cbase;
  //  I2C_Handle       handle;
  //  I2C_Params       i2cparams;
  //  I2C_Transaction  trans;

  uint32_t         device;
  uint32_t         board;
  uint32_t         intline;
  bool            *intflag;


  bool            *switchcmd;
  swRelayPositions *switchnew;
  adCapSelect      cap;
  adCapSelect      cap_prev;

  // State machine
  taskState        state;
  uint32_t         wait;

  // Time since last AD7746 interrupt
  uint32_t         inttime;

  // Count the number of AD7746 capacitance reads
  uint32_t         capreads;

  // Time since last HDC1080 read
  bool             hdc1080initialized;
  uint32_t         temptime;

} taskParams;


EXTERN bool sensor1_ready;
EXTERN bool sensor2_ready;
EXTERN bool sensor3_ready;
EXTERN bool sensor4_ready;
EXTERN bool sensor5_ready;
EXTERN bool sensor6_ready;


/* Name the sensors.
 * Note that sensors are named 1 to 6 and are indexed from 0.
 */
typedef enum {
    SENSOR1 = 0,
    SENSOR2,
    SENSOR3,
    SENSOR4,
    SENSOR5,
    SENSOR6
} sensor_name_t;

typedef struct {
    uint32_t base;
    uint32_t scl;
    uint32_t scl_pin;
    uint32_t sda;
    uint32_t sda_pin;
} sensor_io_t;

/* Define this structure only for use with sensor_task.c, because it is
 * initialized here.  The EXTERN mechanism does not work for initialized
 * values. */
#ifdef SENSOR_TASK_C_
sensor_io_t sensor_io[6] = {
        [SENSOR1].base = SYSCTL_PERIPH_I2C0,
        [SENSOR1].scl = GPIO_PB2_I2C0SCL,
        [SENSOR1].scl_pin = GPIO_PIN_2,
        [SENSOR1].sda = GPIO_PB3_I2C0SDA,
        [SENSOR1].sda_pin = GPIO_PIN_3,

        [SENSOR2].base = SYSCTL_PERIPH_I2C1,
        [SENSOR2].scl = GPIO_PG4_I2C1SCL,
        [SENSOR2].sda = GPIO_PG5_I2C1SDA,

        [SENSOR3].base = SYSCTL_PERIPH_I2C2,
        [SENSOR3].scl = GPIO_PE4_I2C2SCL,
        [SENSOR3].sda = GPIO_PE5_I2C2SDA,
};
#endif


/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */
EXTERN uint32_t Sensor_Task_Init(void);

EXTERN void Sensor1Init(void);
EXTERN void Sensor2Init(void);
EXTERN void Sensor3Init(void);
EXTERN void Sensor4Init(void);
EXTERN void Sensor5Init(void);
EXTERN void Sensor6Init(void);

EXTERN void Sensor1Ready(void);
EXTERN void Sensor2Ready(void);
EXTERN void Sensor3Ready(void);
EXTERN void Sensor4Ready(void);
EXTERN void Sensor5Ready(void);
EXTERN void Sensor6Ready(void);

#undef EXTERN

#endif // SENSOR_TASK_H_
