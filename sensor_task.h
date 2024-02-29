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

#include "includes.h"

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_TASK_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

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
#define AD7746_RESET_REG          0xBF
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




#ifdef hold

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

#endif


/* ----------------------------------------------------------------------------- */
// Task state machine discrete states, separate for each sensor
typedef enum {
    STATE_POR                 = 0,
    STATE_INIT                = 1,
    STATE_INIT_FAILED         = 2,
    STATE_INIT_FAILED_WAIT    = 3,
    STATE_TRIGGER             = 4,
    STATE_TRIGGER_WAIT        = 5,
    STATE_FAULTED             = 6
} sensor_state_t;

/* ----------------------------------------------------------------------------- */
/* PCA9536 state, sets the old/new ACS relay positions */
typedef enum {
    SWITCH_LBL                = 0,
    SWITCH_KONA               = 1
} sensor_relay_position_t;

/* ----------------------------------------------------------------------------- */
/* Single / differential capacitance selection choices */
typedef enum {
    MODE_CAP_DIFFERENTIAL     = 0,
    MODE_CAP_CAP1             = 1,
    MODE_CAP_CAP2             = 2,
    MODE_CAP_MAX              = 3  // Define a maximum value, for looping through the enum
} sensor_cap_mode_t;

#define ADCS_DIFFERENTIAL     0xE0 // CIN2, DIFF=1 (differential capacitance only)
#define ADCS_CAP1             0x80 // CIN1, DIFF=0 (capacitor 1 only)
#define ADCS_CAP2             0xC0 // CIN2, DIFF=0 (capacitor 2 only)

/* ----------------------------------------------------------------------------- */
/* Capacitance conversion times selection */
typedef enum {
    CONVERT_TIME_109MS        = 0, // Default
    CONVERT_TIME_38MS         = 1,
    CONVERT_TIME_11MS         = 2
} sensor_conversion_time_t;

// Configuration register (spec page 18)
// Bit 7: VTF1
// Bit 6: VTF0   (00 = default digital filter setup)
// Bit 5: CAPF2
// Bit 4: CAPF1
// Bit 3: CAPF0  (001 = 11.9ms conversion time, 011 = 38.0ms, 111 = 109.6ms)
// Bit 2: MD2
// Bit 1: MD1
// Bit 0: MD0    (010 = single conversion, 000 = idle)
#define AD7746_CFG_REG        0x0A
#define ADCT_11MS_SINGLE      0b00001010 // 0x0A = 11ms single
#define ADCT_38MS_SINGLE      0b00011010 // 0x1A = 38ms single
#define ADCT_109MS_SINGLE     0b00111010 // 0x3A = 109.6ms single
#define ADCT_109MS_CONT       0b00111001
#define ADCT_109MS_IDLE       0b00111000 // 0x38 = 109.6ms, IDLE


/*
 * This structure contains the sensor state and latest values of the capacitance,
 * temperature, humidity, and chip temperature. *
 */
typedef struct {

    /* State machine */
    sensor_state_t            state;

    /* Timer value for initialization delay (after a failed init) */
    uint32_t                  init_wait;

    /* Sensor switching */
    sensor_relay_position_t   relay_position;

    /* Capacitor selection */
    sensor_cap_mode_t         cap_mode;

    /* Capacitor conversion time */
    sensor_conversion_time_t  conversion_time;

} sensor_control_t;

EXTERN sensor_control_t sensor_control[MAX_SENSORS];



/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */

EXTERN bool Sensor_Reset(sensor_name_t sensor);
EXTERN bool Sensor_Init(sensor_name_t sensor);
EXTERN bool Sensor_Trigger(sensor_name_t sensor);
EXTERN void Sensor_Process(sensor_name_t sensor);
EXTERN uint32_t Sensor_Task_Init(void);

#undef EXTERN

#endif // SENSOR_TASK_H_
