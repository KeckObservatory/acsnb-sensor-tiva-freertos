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





/* -----------------------------------------------------------------------------
 * AD7746 - Capacitance sensor
 */

/* Base address of device on I2C bus */
#define AD7746_ADDR               0x48

/* Commands */
#define AD7746_WRITE              0x00
#define AD7746_READ               0x01

/* Registers and their definitions */
#define AD7746_RESET_REG          0xBF
#define AD7746_STATUS_REG         0x00

/* Capacitance setup register definitions (spec pg 16)
 *
 * Bit 7: CAPEN   (1 = enables capacitive channel for single conversion, continuous conversion, or calibration)
 * Bit 6: CIN2    (1 = switches the internal multiplexer to the second capacitive input on the AD7746)
 * Bit 5: CAPDIFF (1 = sets differential mode on the selected capacitive input)
 * Bit 4-1: Set to 0.
 * Bit 0: CAPCHOP (1 = approximately doubles the capacitive channel conversion times and slightly improves the
                       capacitive channel noise performance for the longest conversion times)
 */
#define AD7746_CAP_SETUP_REG      0x07
#define AD7746_CAP_DIFFERENTIAL   0b11100000 // 0xE0 = CIN2, DIFF=1 (differential capacitance only)
#define AD7746_CAP_CAP1           0b10000000 // 0x80 = CIN1, DIFF=0 (capacitor 1 only)
#define AD7746_CAP_CAP2           0b11000000 // 0xC0 = CIN2, DIFF=0 (capacitor 2 only)

/* Voltage setup register definitions (spec pg 16)
 *
 * Bit 7: VTEN (1 = enables voltage/temperature channel for single conversion)
 * Bit 6-5: VTMD1, VTMD0 Channel Input (00 = Internal temperature sensor)
 * Bit 4: EXTREF (0 = select the on-chip internal reference)
 * Bit 3-2: (00 - These bits must be 0 for proper operation)
 * Bit 1: VTSHORT (0 = disable internal short of the voltage/temperature channel input for test purposes)
 * Bit 0 VTCHOP (1 = sets internal chopping on the voltage/temperature channel / must be set to 1 for the specified voltage/temperature channel performance)
 */
#define AD7746_VT_SETUP_REG       0x08
#define AD7746_VT_SETUP_DISABLE   0x00
#define AD7746_VT_SETUP_INT_TEMP  0b10000001 // 0x81 = VTEN=1, VTCHOP=1

/* Excitation setup register definitions (spec pg 17)
 * Bit 7: CLKCTRL (0 = default, 1 = decrease frequencies by factor of 2)
 * Bit 6: EXCON   (1 = excitation signal present on output during cap channel conversion AND during voltage/temp conversion)
 * Bit 5: EXCB    (0 = disable EXCB pin as the excitation output)
 * Bit 4: NOTEXCB (0 = disable EXCB pin as the inverted excitation output)
 * Bit 3: EXCA    (1 = enable EXCA pin as the excitation output)
 * Bit 2: NOTEXCA (0 = disable EXCA pin as the inverted excitation output)
 * Bit 1,0: EXCLV1 EXCLV0 (excitation voltage level)
 *          11 = Voltage on cap     = (+/- Vdd)/2
 *               EXC Pin Low Level  = 0
 *               EXC Pin High Level = Vdd
 */
#define AD7746_EXC_SETUP_REG      0x09
#define AD7746_EXC_SET_A          0b01001011 // EXCON=1, EXCA=1, EXCLV=1, EXCLV0=1

/* Configuration register (spec page 18)
 * Bit 7: VTF1
 * Bit 6: VTF0   (00 = default digital filter setup)
 * Bit 5: CAPF2
 * Bit 4: CAPF1
 * Bit 3: CAPF0  (001 = 11.9ms conversion time, 011 = 38.0ms, 111 = 109.6ms)
 * Bit 2: MD2
 * Bit 1: MD1
 * Bit 0: MD0    (010 = single conversion, 000 = idle)
 */
#define AD7746_CFG_REG            0x0A
#define AD7746_CFG_C_11MS_SINGLE  0b00001010 // 0x0A = 11ms single cap convert
#define AD7746_CFG_C_38MS_SINGLE  0b00011010 // 0x1A = 38ms single cap convert
#define AD7746_CFG_C_109MS_SINGLE 0b00111010 // 0x3A = 109.6ms single cap convert
#define AD7746_CFG_T_20MS_SINGLE  0b00000010 // 0x02 = 20ms single temp convert
#define AD7746_CFG_T_32MS_SINGLE  0b01000010 // 0x42 = 32ms single temp convert
#define AD7746_CFG_T_62MS_SINGLE  0b10000010 // 0x82 = 62ms single temp convert

#define AD7746_CFG_T_DEFAULT      AD7746_CFG_T_32MS_SINGLE
#define AD7746_TEMP_TRIGGER_RATE  10   // Trigger one temperature read every 10 cap reads



// -----------------------------------------------------------------------------
// PCA9536 - Relay driver to switch back to old ACS connection
#define PCA9536_ADDR              0x41
#define PCA9536_OUT_PORT_REG      0x01
#define PCA9536_OUT_PORT_RESET    0x00
#define PCA9536_OUT_PORT_NEW_ACS  0x05
#define PCA9536_OUT_PORT_OLD_ACS  0x0A
#define PCA9536_CONFIG_REG        0x03
#define PCA9536_CONFIG_ALL_OUTPUT 0x00


/* ----------------------------------------------------------------------------- */
// Task state machine discrete states, separate for each sensor
typedef enum {
    STATE_POR                   = 0,
    STATE_INIT                  = 1,
    STATE_INIT_FAILED           = 2,
    STATE_INIT_FAILED_WAIT      = 3,
    STATE_TRIGGER_CAP           = 4,
    STATE_TRIGGER_CAP_WAIT      = 5,
    STATE_TRIGGER_TEMPERATURE   = 6,
    STATE_TRIGGER_TEMP_WAIT     = 7,
    STATE_FAULTED               = 8,
    STATE_MAX
} sensor_state_t;

/* ----------------------------------------------------------------------------- */
/* PCA9536 state, sets the old/new ACS relay positions */
typedef enum {
    SWITCH_LBL                  = 0,
    SWITCH_KONA                 = 1
} sensor_relay_position_t;

/* ----------------------------------------------------------------------------- */
/* Single / differential capacitance, or temperature, selection choices */
typedef enum {
    MODE_C_DIFFERENTIAL       = 0,
    MODE_C_CAP1               = 1,
    MODE_C_CAP2               = 2,
    MODE_TEMPERATURE          = 3
} sensor_mode_t;

/* ----------------------------------------------------------------------------- */
/* Capacitance conversion times selection */
typedef enum {
    CONVERT_TIME_109MS          = 0, // Default
    CONVERT_TIME_38MS           = 1,
    CONVERT_TIME_11MS           = 2
} sensor_conversion_time_t;





#ifdef hold

// Task state data
typedef struct {

  bool            *switchcmd;
  swRelayPositions *switchnew;
  adCapSelect      cap;
  adCapSelect      cap_prev;

  // Time since last AD7746 interrupt
  uint32_t         inttime;

  // Count the number of AD7746 capacitance reads
  uint32_t         capreads;

  // Time since last HDC1080 read
  bool             hdc1080initialized;
  uint32_t         temptime;

} taskParams;

#endif


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

    /* Capacitor/temperature selection mode */
    sensor_mode_t             mode;

    /* Flag that enables retrieving single-ended caps alongside differential;
     * used only for debugging because it runs the differential cap retrieval
     * at one third the usual sampling rate. */
    bool                      enable_c1_c2;

    /* Capacitor conversion time */
    sensor_conversion_time_t  conversion_time;

    /* Capacitor conversion cycles, used to interleave temperature measurements */
    uint8_t                   conversions;

} sensor_control_t;

EXTERN sensor_control_t sensor_control[MAX_SENSORS];



/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */

EXTERN bool Sensor_Reset(sensor_name_t sensor);
EXTERN bool Sensor_Init(sensor_name_t sensor);
EXTERN bool Sensor_Trigger(sensor_name_t sensor, sensor_mode_t cap_mode);
EXTERN bool Sensor_Read(sensor_name_t sensor, sensor_mode_t cap_mode);
EXTERN void Sensor_Process(sensor_name_t sensor);
EXTERN uint32_t Sensor_Task_Init(void);

#undef EXTERN

#endif // SENSOR_TASK_H_
