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

/* -----------------------------------------------------------------------------
 * Timer values.
 */

/* Attempt to re-initialize the sensor once a second */
#define SENSOR_INIT_TIMEOUT_MS    1000

/* Timeout value for the ready signal from the sensor */
#define SENSOR_READY_TIMEOUT_MS   500

/* Attempt to init the temp+humidity sensor every 10 seconds */
#define TH_REINIT_TIMEOUT_MS      1000 //10000



/* -----------------------------------------------------------------------------
 * SI7020 Temperature / Humidity sensor
 */
#define SI7020_ADDR          0x40
#define SI7020_HUM_HOLD      0xE5
#define SI7020_HUM_NO_HOLD   0xF5
#define SI7020_TMP_HOLD      0xE3
#define SI7020_TMP_NO_HOLD   0xF3
#define SI7020_TMP_PREVIOUS  0xF0
#define SI7020_RESET         0xFE
#define SI7020_WRITE_USER_1  0xE6
#define SI7020_WRITE_USER_2  0x51
#define SI7020_READ_HEATER   0x11

/* 2 byte register addresses to read the ESN */
#define SI7020_READ_ESN1_1   0xFA
#define SI7020_READ_ESN1_2   0x0F
#define SI7020_READ_ESN2_1   0xFC
#define SI7020_READ_ESN2_2   0xC9

/* Offsets into the buffers from the ESN reads to get the individual
 * bytes of the ESN.  They do not arrive sequentially.  See Si7020
 * spec, page 24. */
#define SI7020_SNA_0         6
#define SI7020_SNA_1         4
#define SI7020_SNA_2         2
#define SI7020_SNA_3         0

#define SI7020_SNB_0         4
#define SI7020_SNB_1         3
#define SI7020_SNB_2         1
#define SI7020_SNB_3         0

#define SI7013_ID            0x0D
#define SI7020_ID            0x14
#define SI7021_ID            0x15

/*
 * Si7020 temperature is calculated from two bytes, t0 and t1.
 * T = (T0 << 8 + T1) * 175.72 / 65536-46.85
 *
 * Humidity is expressed as:
 * H = (H0 << 8 + H1) * 125 / 65536 - 6
 *
 * When the sensor is disconnected, we want to read back invalid values, such as a
 * temperatures of 999C and humidity of 0%.  Back calculate the values to return
 * when this is the case.
 *
 */

/* 99C is equal to 54396 (decimal) */
#define SI7020_INVALID_TH    0xD4
#define SI7020_INVALID_TL    0x7C

/* 0% RH is equal to 3146 (decimal) */
#define SI7020_INVALID_HH    0x0C
#define SI7020_INVALID_HL    0x4A

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
    STATE_IDLE                  = 1,
    STATE_RESET                 = 2,
    STATE_INIT                  = 3,
    STATE_TRIGGER_CAP           = 4,
    STATE_TRIGGER_CAP_WAIT      = 5,
    STATE_TRIGGER_TEMPERATURE   = 6,
    STATE_TRIGGER_TEMP_WAIT     = 7,
    STATE_READ_TH               = 8,
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

    /* Disabled for use flag. Not all sensors need to be connected, some
     * mirror segments have fewer than 6 sensors on them.  Note that this
     * is called "disabled" instead of "enabled" so that the default value
     * will be to enable all the sensors, and the node box software can
     * selectively disable sensors as needed. */
    bool                      disabled;

    /* Connection status */
    bool                      ad7746_connected;

    /* Timer value for initialization delay (after a failed init) */
    int32_t                   init_wait_timer;

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

    /* Si7020 temperature+humidity sensing control */
    bool                      si7020_connected;
    uint8_t                   si7020_esn[8];
    int32_t                   si7020_timer;

} sensor_control_t;

EXTERN sensor_control_t sensor_control[MAX_SENSORS];



/* -----------------------------------------------------------------------------
 * Function prototypes
 */

EXTERN bool Sensor_Reset(sensor_name_t sensor);
EXTERN bool Sensor_Init(sensor_name_t sensor);
EXTERN bool Sensor_Trigger(sensor_name_t sensor, sensor_mode_t cap_mode);
EXTERN bool Sensor_Read(sensor_name_t sensor, sensor_mode_t cap_mode);
EXTERN void Sensor_Process(sensor_name_t sensor);
EXTERN uint32_t Sensor_Task_Init(void);

#undef EXTERN

#endif // SENSOR_TASK_H_
