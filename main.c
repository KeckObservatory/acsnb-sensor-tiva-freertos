/*
 * main.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

// Board support includes
#include <inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_i2c.h>

#include <driverlib/interrupt.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/ssi.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <utils/uartstdio.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "priorities.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Tasks
#include "spi_task.h"
#include "sensor_task.h"



// -----------------------------------------------------------------------------
// High level defines

// Firmware revision as of 2023-12-11 (PMR)
#define FIRMWARE_REV_0 1
#define FIRMWARE_REV_1 0
#define FIRMWARE_REV_2 0

#define MAX_SENSORS               6

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

// Signature pattern to determine if it's a real message
#define SIGNATURE0               (0xA5)
#define SIGNATURE1               (0x5A)

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
adConversionTime adAllSensorConversionTime = DEFAULT_CONVERSION_TIME;


/* Single / differential capacitance selection choices */
typedef enum {
  //adcsC1D1                      = 0xA0, // CIN1, DIFF=1
  adcsC2D1                      = 0xE0, // CIN2, DIFF=1
  adcsC1D0                      = 0x80, // CIN1, DIFF=0
  adcsC2D0                      = 0xC0  // CIN2, DIFF=0

} adCapSelect;

#define DEFAULT_CAPACITOR_SELECT adcsC2D1

// Flags to indicate whether to only get the differential cap, or get all 3 (for each sensor)
bool adGetAllCaps[MAX_SENSORS] = { false, false, false, false, false, false };


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

// -----------------------------------------------------------------------------
// SPI messaging

// Define a structure which represents the data going back down the SPI, contains
// all the values of capacitance and temperature and humidity.  Packing
// is used here to prevent any padding that might be inserted by the compiler.
union spiMessageOut_u {

  struct {

      // 5 bytes of Header information first
      uint8_t signature0;
      uint8_t signature1;
      uint8_t version0;
      uint8_t version1;
      uint8_t version2;

      struct {

        // Bytes 0 and 1 are the temperature
        uint8_t humidityHigh;
        uint8_t humidityLow;

        // Bytes 2, 3 and 4 are the differential capacitance, a 24 bit value
        uint8_t diffCapHigh;
        uint8_t diffCapMid;
        uint8_t diffCapLow;

        // Bytes 5, 6 and 7 are the C1 cap single capacitance
        uint8_t c1High;
        uint8_t c1Mid;
        uint8_t c1Low;

        // Bytes 8, 9 and 10 are the C2 cap single capacitance
        uint8_t c2High;
        uint8_t c2Mid;
        uint8_t c2Low;

        // Byte 11, 12 and 13 are the filtered differential capacitance, a 24 bit value
        uint8_t filtCapHigh;
        uint8_t filtCapMid;
        uint8_t filtCapLow;

        // Bytes 14, 15 are the temperature
        uint8_t tempHigh;
        uint8_t tempLow;

        // Bytes 16, 17, and 18 are the on-chip temperature from the capacitance sensor
        uint8_t chiptempHigh;
        uint8_t chiptempMid;
        uint8_t chiptempLow;

      } sensor[MAX_SENSORS];

  } msg;

  uint8_t buf[5 + (MAX_SENSORS * 19)];

} __attribute__((packed));

typedef union spiMessageOut_u spiMessageOut_t;

spiMessageOut_t spiMessageOut;

#define SPI_MESSAGE_LENGTH sizeof(spiMessageOut)

union spiMessageIn_u {
  struct {

    /* First 4 bytes are used for commanding the device */
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t cmd2;
    uint8_t cmd3;

    /* Subsequent bytes are for settings that are broadcast every messaging cycle */
    uint8_t useFastConversionTime;
  };

  /* Make the input buffer match the size of the output by mapping an array on top of it */
  uint8_t buf[sizeof(spiMessageOut)];

} __attribute__((packed));

typedef union spiMessageIn_u spiMessageIn_t;

spiMessageIn_t spiMessageIn;


// -----------------------------------------------------------------------------
// Switch states

// Current switch command; set this flag to trigger the thread to switch its value
bool switchcmd0 = false;
bool switchcmd1 = false;
bool switchcmd2 = false;
bool switchcmd3 = false;
bool switchcmd4 = false;
bool switchcmd5 = false;

// New switch value
swRelayPositions switchNew0 = swNewACS;
swRelayPositions switchNew1 = swNewACS;
swRelayPositions switchNew2 = swNewACS;
swRelayPositions switchNew3 = swNewACS;
swRelayPositions switchNew4 = swNewACS;
swRelayPositions switchNew5 = swNewACS;

// Interrupt flags, one for each sensor
bool intflag0 = false;
bool intflag1 = false;
bool intflag2 = false;
bool intflag3 = false;
bool intflag4 = false;
bool intflag5 = false;

int currentSwitchPosition = PCA9536_OUT_PORT_NEW_ACS;
// -----------------------------------------------------------------------------
// Task control structure


//*****************************************************************************
// The mutex that protects concurrent access of UART from multiple tasks.
//*****************************************************************************
//xSemaphoreHandle g_pUARTSemaphore;

// Semaphore for locking the message structure
//Semaphore_Struct semStruct;
xSemaphoreHandle semHandle;


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





//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line) { }
#endif

//*****************************************************************************
// This hook is called by FreeRTOS when an stack overflow error is detected.
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {

    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.

    while(1) {
    }
}

//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//*****************************************************************************
void ConfigureUART(void) {

    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}


void InitI2C0(void) {
    // Initialize I2C module 0

    // Enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // Enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // Clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void I2CSend(uint8_t slave_addr, uint8_t num_of_args, ...) {

    uint8_t i;

    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    // Stores list of variable number of arguments
    va_list vargs;

    // Specifies the va_list to "open" and the last fixed argument
    // so vargs knows where to start looking
    va_start(vargs, num_of_args);

    // Put data to be sent into FIFO
    I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

    // If there is only one argument, we only need to use the
    // single send I2C function
    if(num_of_args == 1) {

        // Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // "close" variable argument list
        va_end(vargs);

    // Otherwise, we start transmission of multiple bytes on the I2C bus
    } else {

        // Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // Send num_of_args-2 pieces of data, using the BURST_SEND_CONT command of the I2C module
        for(i = 1; i < (num_of_args - 1); i++) {

            // Put next piece of data into I2C FIFO
            I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

            // Send next data that was just placed into FIFO
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

            // Wait until MCU is done transferring.
            while(I2CMasterBusy(I2C0_BASE));
        }

        // Put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, va_arg(vargs, uint32_t));

        // Send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Wait until MCU is done transferring.
        while(I2CMasterBusy(I2C0_BASE));

        // "close" variable args list
        va_end(vargs);
    }
}

uint8_t I2CReceive1(uint32_t slave_addr, uint8_t reg) {

    // Specify that we are writing (a register address) to the slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    // Specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    // Send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    // Send control byte and read from the register we specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Return one byte pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

uint16_t I2CReceive2(uint32_t slave_addr, uint8_t reg) {

    uint8_t msb, lsb;

    // Specify that we are writing (a register address) to the slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    // Specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    // Send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    // Send control byte and read from the register we specified
    I2CMasterBurstLengthSet(I2C0_BASE, 0x2);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Capture the MSB
    msb = I2CMasterDataGet(I2C0_BASE);

    //I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    // Capture the LSB
    lsb = I2CMasterDataGet(I2C0_BASE);

    // Assemble the uint16 from MSB+LSB
    return (msb << 8) | lsb;
}



//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void) {
    //uint32_t temphum = 0;

    /* Set the clocking to run at 80 MHz from the PLL */
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    /* Initialize the UART and configure it for 115,200, 8-N-1 operation */
    ConfigureUART();

    /* Initialize the I2C busses (6 of them) */
    InitI2C0();

    SysCtlPeripheralClockGating(true);

    UARTprintf("\n\nACS Node Box sensor module started\n");

    //temphum = I2CReceive(0x40, 0xE3);
    //UARTprintf("temphum = %d\n", temphum);

    // Create a mutex to guard the messaging.
    semHandle = xSemaphoreCreateMutex();

    // Create the sensor task.
    if(Sensor_Task_Init() != 0) {
        while(1) {
        }
    }

    if(SPI_Task_Init() != 0) {
        while(1) {
        }
    }







    /*
    // Init Interrupt sensor 0
    GPIO_disableInt(Board_PININ0);
    GPIO_clearInt(Board_PININ0);1
    GPIO_setCallback(Board_PININ0, sens0cvtDoneItr);

    // Init Interrupt sensor 1
    GPIO_disableInt(Board_PININ1);
    GPIO_clearInt(Board_PININ1);
    GPIO_setCallback(Board_PININ1, sens1cvtDoneItr);

    // Init Interrupt sensor 2
    GPIO_disableInt(Board_PININ2);
    GPIO_clearInt(Board_PININ2);
    GPIO_setCallback(Board_PININ2, sens2cvtDoneItr);

    // Init Interrupt sensor 3
    GPIO_disableInt(Board_PININ3);
    GPIO_clearInt(Board_PININ3);
    GPIO_setCallback(Board_PININ3, sens3cvtDoneItr);

    // Init Interrupt sensor 4
    GPIO_disableInt(Board_PININ4);
    GPIO_clearInt(Board_PININ4);
    GPIO_setCallback(Board_PININ4, sens4cvtDoneItr);

    // Init Interrupt sensor 5
    GPIO_disableInt(Board_PININ5);
    GPIO_clearInt(Board_PININ5);
    GPIO_setCallback(Board_PININ5, sens5cvtDoneItr);
    */



    // Enable the GPIO port that is used for the on-board LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3); // on
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0); // off



    /* All tasks are primed, enable interrupts */
    IntMasterEnable();

    /* Start the scheduler.  This should not return. */
    vTaskStartScheduler();

    /* In case the scheduler returns for some reason, print an error and loop forever */
    while(1) {
    }
}
