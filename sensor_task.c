/*
 * sensor_task.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define SENSOR_TASK_C_
#include "includes.h"

#define SENSOR_TASK_STACK_SIZE     128  // Stack size in words
#define SENSOR_ITEM_SIZE           sizeof(uint8_t)
#define SENSOR_QUEUE_SIZE          5


adConversionTime adAllSensorConversionTime = DEFAULT_CONVERSION_TIME;

// Flags to indicate whether to only get the differential cap, or get all 3 (for each sensor)
bool adGetAllCaps[MAX_SENSORS] = { false, false, false, false, false, false };

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

/* -----------------------------------------------------------------------------
 *
 */
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

/* -----------------------------------------------------------------------------
 *
 */
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

/* -----------------------------------------------------------------------------
 *
 */
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


/* -----------------------------------------------------------------------------
 *
 */
static void Sensor_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t sensorTaskDelay;
    int32_t heartbeatTimer;

    sensorTaskDelay = 250;
    bool toggle;

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Count down 500ms for each heartbeat LED change.  Tick rate is 1KHz, which is 1ms per tick.
    heartbeatTimer = 500;

    // UART semaphore example
    //xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    //UARTprintf("Led blinking frequency is %d ms.\n", (ui32LEDToggleDelay * 2));
    //xSemaphoreGive(g_pUARTSemaphore);



    // Loop forever.
    while (1) {

        /* This will be invoked once per millisecond */
        heartbeatTimer--;

        if (heartbeatTimer < 0) {

            if (toggle) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // off
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // off
            }

            toggle = !toggle;

            heartbeatTimer = 500;

            v_printf("heartbeat\n");
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, sensorTaskDelay / portTICK_RATE_MS);
    }
}

/* -----------------------------------------------------------------------------
 *
 */
uint32_t Sensor_Task_Init(void) {





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



    if(xTaskCreate(Sensor_Task, (const portCHAR *)"SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}
