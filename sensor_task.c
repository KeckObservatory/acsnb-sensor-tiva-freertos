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

#ifdef hold
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

int currentSwitchPosition = PCA9536_OUT_PORT_NEW_ACS;
#endif


//*****************************************************************************
//! Indicates whether or not the I2C bus has timed out.
//!
//! \param ui32Base is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C bus has time
//!  out.  The I2C Master Timeout Value must be set.
//!
//! \return Returns \b true if the I2C bus has timed out; otherwise, returns
//! \b false.
//*****************************************************************************
bool I2CMasterTimeout(uint32_t ui32Base) {

    // Return the bus timeout status
    if (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_CLKTO) {
       return(true);
    } else {
       return(false);
    }
}

/* -----------------------------------------------------------------------------
 * Initialize sensor port 1: I2C module 0 (SDA0, SCL0) and RDY_1_OUT on PA7
 */
void I2CInit(sensor_name_t sensor) {

    uint32_t peripheral = sensor_io[sensor].peripheral;
    uint32_t periph_base = sensor_io[sensor].periph_base;
    uint32_t port_base = sensor_io[sensor].port_base;
    uint32_t scl = sensor_io[sensor].scl;
    uint32_t scl_pin = sensor_io[sensor].scl_pin;
    uint32_t sda = sensor_io[sensor].sda;
    uint32_t sda_pin = sensor_io[sensor].sda_pin;
    uint32_t rdy_port = sensor_io[sensor].rdy_port;
    uint32_t rdy_pin = sensor_io[sensor].rdy_pin;
    isrFunc isr = sensor_io[sensor].isr;
    bool *isr_flag = sensor_io[sensor].isr_flag;

    /* Enable the I2C peripheral */
    SysCtlPeripheralEnable(peripheral);
    while (!SysCtlPeripheralReady(peripheral)) {}

    /* Reset peripheral */
    SysCtlPeripheralReset(peripheral);

    /* Configure the pin muxing for I2C0 functions on port B2 and B3. */
    GPIOPinConfigure(scl);
    GPIOPinConfigure(sda);

    /* Select the I2C function for these pins. */
    GPIOPinTypeI2CSCL(port_base, scl_pin);
    GPIOPinTypeI2C(port_base, sda_pin);

    /*
     * Enable and initialize the I2C0 master module.  Use the system clock for
     * the I2C0 module.  The last parameter sets the I2C data transfer rate.
     * If false the data rate is set to 100kbps and if true the data rate will
     * be set to 400kbps.
     */
    I2CMasterInitExpClk(periph_base, SysCtlClockGet(), false);

    /* TBD: use the glitch filter if the bus is noisy */
    //I2CMasterGlitchFilterConfigSet(periph_base, I2C_MASTER_GLITCH_FILTER_32);

    /* Set a bus timeout value just in case a message is disrupted/corrupted by an
     * unplug event.  Use the max value to avoid trying to predict how long we need. */
    I2CMasterTimeoutSet(periph_base, 0xFF);

    /* Clear I2C FIFOs */
    HWREG(periph_base + I2C_O_FIFOCTL) = 80008000;

    /* Connect an interrupt handler to the ready select pin. */
    GPIOPinTypeGPIOInput(rdy_port, rdy_pin);
    GPIOIntRegister(rdy_port, isr);

    /* From the AD7745 spec pg 7: A falling edge on this output indicates that a
     * conversion on enabled channel(s) has been finished and the new data is available.
     */
    GPIOIntTypeSet(rdy_port, rdy_pin, GPIO_FALLING_EDGE);
    GPIOIntEnable(rdy_port, rdy_pin);

    /* Clear the ready flag for the device */
    *isr_flag = false;
}

/* -----------------------------------------------------------------------------
 * Interrupt handler for sensor 1 conversion ready signal.
 */
void Sensor1Ready(void) {

    /* Clear the interrupt */
    GPIOIntClear(sensor_io[SENSOR1].rdy_port, sensor_io[SENSOR1].rdy_pin);

    /* Set the flag that the conversion is complete */
    *sensor_io[SENSOR1].isr_flag = true;
}

/* -----------------------------------------------------------------------------
 * Interrupt handlers for sensors 2-6 conversion ready signal.  Same code as
 * above but comments are removed for compactness.
 */
void Sensor2Ready(void) {
    GPIOIntClear(sensor_io[SENSOR2].rdy_port, sensor_io[SENSOR2].rdy_pin);
    *sensor_io[SENSOR2].isr_flag = true;
}
void Sensor3Ready(void) {
    GPIOIntClear(sensor_io[SENSOR3].rdy_port, sensor_io[SENSOR3].rdy_pin);
    *sensor_io[SENSOR3].isr_flag = true;
}
void Sensor4Ready(void) {
    GPIOIntClear(sensor_io[SENSOR4].rdy_port, sensor_io[SENSOR4].rdy_pin);
    *sensor_io[SENSOR4].isr_flag = true;
}
void Sensor5Ready(void) {
    GPIOIntClear(sensor_io[SENSOR5].rdy_port, sensor_io[SENSOR5].rdy_pin);
    *sensor_io[SENSOR5].isr_flag = true;
}
void Sensor6Ready(void) {
    GPIOIntClear(sensor_io[SENSOR6].rdy_port, sensor_io[SENSOR6].rdy_pin);
    *sensor_io[SENSOR6].isr_flag = true;
}



/* -----------------------------------------------------------------------------
 * Send a number of bytes to an I2C address.
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



int8_t I2CReceive(uint32_t base, uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len) {

    uint8_t i;
    uint32_t err;

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        /*
        if (err == I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err == I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err == I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err == I2C_MASTER_ERR_CLK_TOUT)
            return -4;
        */
    }



    /* Specify that we are writing (a register address) to the slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, false);

    /* Specify register to be read */
    I2CMasterDataPut(base, reg);

    /* Send control byte and register address byte to slave device */
    I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);

    /* Wait for MCU to finish transaction */
    while(I2CMasterBusy(base));

    /* Specify that we are going to read from slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, true);

    /* Send control byte and read from the register we specified */
    I2CMasterBurstLengthSet(base, len);

    for (i = 0; i < len; i++) {

        if (i == 0) {
            /* First byte starts the transaction */
            I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
        } else if (i == (len-1)) {
            /* Last byte finishes the transaction */
            I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        } else {
            I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        }

        /* Wait for MCU to finish transaction */
        while(I2CMasterBusy(base));

        /* Capture one byte */
        buf[i] = I2CMasterDataGet(base);
    }

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err == I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err == I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err == I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err == I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    return 0;
}












#ifdef hold



/*
 *  ======== setupAD7746 ========
 *
 */
int setupAD7746(I2C_Handle i2c, I2C_Transaction i2cTransaction, uint8_t device)
{
  uint8_t txBuffer[2];
  uint8_t rxBuffer[4];
  uint8_t offsH, offsL, gainH, gainL;

  Task_sleep(100);

  // Configure CAPACITANCE MEASUREMENT
  // -----------------------------------------------
  txBuffer[0]                 = AD7746_CAP_SETUP_REG;
  txBuffer[1]                 = adcsC2D1;
  i2cTransaction.slaveAddress = AD7746_ADDR;
  i2cTransaction.writeBuf     = txBuffer;
  i2cTransaction.writeCount   = 2;
  i2cTransaction.readBuf      = rxBuffer;
  i2cTransaction.readCount    = 0;

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in setup of AD7746 (default capacitors).\n", device);
    System_flush();
    return -1;
  }
  System_flush();

  Task_sleep(100);

  // Configure VOLTAGE/TEMPERATURE (enable internal temperature sensor)
  // -----------------------------------------------
  txBuffer[0] = AD7746_VT_SETUP_REG;
  txBuffer[1] = AD7746_VT_SETUP_INT_TEMP;

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in setup of AD7746 (setup for temperature reading).\n", device);
    System_flush();
    return -1;
  }
  System_flush();

  Task_sleep(100);

  // Configure EXCITATION
  // -----------------------------------------------
  txBuffer[0] = AD7746_EXC_SETUP_REG;
  txBuffer[1] = AD7746_EXC_SET_A;

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in setup of AD7746 (configuring excitation).\n", device);
    System_flush();
    return -1;
  }
  System_flush();

  Task_sleep(100);

  // Configure CONVERSION TIME
  // -----------------------------------------------
  txBuffer[0] = AD7746_CFG_REG;
  txBuffer[1] = adAllSensorConversionTime;

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in setup of AD7746 (setting conversion time).\n", device);
    System_flush();
    return -1;
  }
  System_flush();

  Task_sleep(100);


  // Read CAPACITATIVE OFFSET CALIBRATION/GAIN
  // -----------------------------------------------

  /* Read AD7746 register starting at Cap Offset H, total of 4 bytes */
  txBuffer[0]                 = AD7746_CAP_OFFSET_H;
  i2cTransaction.writeBuf     = txBuffer;
  i2cTransaction.writeCount   = 1;
  i2cTransaction.readBuf      = rxBuffer;
  i2cTransaction.readCount    = 4;

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in setup of AD7746 (reading calibration).\n", device);
    System_flush();
    return -1;
  }

  offsH = rxBuffer[0];
  offsL = rxBuffer[1];
  gainH = rxBuffer[2];
  gainL = rxBuffer[3];

  System_printf("(%d) Calibrations: offset H: %d, offset L: %d, gain H: %d, gain L: %d\n", device, offsH, offsL, gainH, gainL);
  System_flush();

  return 0;
}



/*
 *  ======== triggerAD7746capacitance ========
 *
 */
int triggerAD7746capacitance(I2C_Handle i2c, I2C_Transaction i2cTransaction, adConversionTime convTim, adCapSelect cap, uint8_t device) {

    uint8_t txBuffer[2];
    uint8_t rxBuffer[4];

    /* Common message setup fields */
    i2cTransaction.slaveAddress = AD7746_ADDR;
    i2cTransaction.writeBuf     = txBuffer;
    i2cTransaction.writeCount   = 2;
    i2cTransaction.readBuf      = rxBuffer;
    i2cTransaction.readCount    = 0;

    /* Build first message to device: set capacitor configuration */
    txBuffer[0] = AD7746_CAP_SETUP_REG;
    txBuffer[1] = cap;

    if (!I2C_transfer(i2c, &i2cTransaction)) {
      System_printf("(%d) Error in AD7746 trigger (cap selection) of AD7746.\n", device);
      System_flush();
      return -1;
    }

    /* Build second message to device: set conversion time and trigger conversion */
    txBuffer[0] = AD7746_CFG_REG;
    txBuffer[1] = convTim;

    if (!I2C_transfer(i2c, &i2cTransaction)) {
      System_printf("(%d) Error in AD7746 trigger (set conversion time) of AD7746.\n", device);
      System_flush();
      return(-1);
    }

    return 0;
}


/*
 *  ======== triggerAD7746 ========
 *
 */
int triggerAD7746temperature(I2C_Handle i2c, I2C_Transaction i2cTransaction, uint8_t device) {

    uint8_t txBuffer[2];
    uint8_t rxBuffer[4];

    /* Common message setup fields */
    i2cTransaction.slaveAddress = AD7746_ADDR;
    i2cTransaction.writeBuf     = txBuffer;
    i2cTransaction.writeCount   = 2;
    i2cTransaction.readBuf      = rxBuffer;
    i2cTransaction.readCount    = 0;

    //TODO: Probably don't need this since it gets enabled above?
    /* Build message to device, read the temperature */
    /*
    txBuffer[0] = AD7746_VT_SETUP_REG;
    txBuffer[1] = AD7746_VT_SETUP_INT_TEMP;

    if (!I2C_transfer(i2c, &i2cTransaction)) {
      System_printf("(%d) Error in AD7746 trigger (temperature enable) of AD7746.\n", device);
      System_flush();
      return -1;
    }
    */

    /* Build message to device: set conversion time and trigger conversion */
    txBuffer[0] = AD7746_CFG_REG;
    txBuffer[1] = DEFAULT_TEMPERATURE_CONVERSION_TIME;

    if (!I2C_transfer(i2c, &i2cTransaction)) {
      System_printf("(%d) Error in AD7746 trigger (set temperature conversion time) of AD7746.\n", device);
      System_flush();
      return(-1);
    }

    return 0;
}

/*  ======== readAD7746 ========
 *  function to read AD7746 capacitance & temperature
 *
 */
int readAD7746(I2C_Handle i2c, I2C_Transaction i2cTransaction, adCapSelect cap, uint8_t device) {

  uint8_t txBuffer[1];
  uint8_t rxBuffer[6];
  uint32_t ci;
  float cr,c;

  /* Read Ad7746 */
  txBuffer[0] = AD7746_READ;
  i2cTransaction.slaveAddress = AD7746_ADDR;
  i2cTransaction.writeBuf     = txBuffer;
  i2cTransaction.writeCount   = 1;
  i2cTransaction.readBuf      = rxBuffer;
  i2cTransaction.readCount    = 6; // 3 bytes for cap only, 6 for cap and temp (see spec page 14)

  if (!I2C_transfer(i2c, &i2cTransaction)) {
    System_printf("(%d) Error in reading AD7746.\n", device);
    System_flush();
    return -1;
  }

  /* Get access to resource */
  Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

  // Put the values into the buffer used to talk back up the SPI
  switch(cap) {

    // Differential capacitor value
    case adcsC2D1:
      spiMessageOut.msg.sensor[device].diffCapHigh = rxBuffer[0];
      spiMessageOut.msg.sensor[device].diffCapMid  = rxBuffer[1];
      spiMessageOut.msg.sensor[device].diffCapLow  = rxBuffer[2];

      // Calculate the capacitance as a float, for usage in the filtered value
      cr = (float) ((rxBuffer[0] << 16) + (rxBuffer[1] << 8) + (rxBuffer[2]));
      c  = -4.096 + (cr * 8.192 / (1 << 24));

      // Apply filtering algorithm to the value and the previous values
      filter[device].c = (FILTER_COEFF * filter[device].cprev) + ((1.0 - FILTER_COEFF) * c);

      // Drop the last values down to the 'previous' value position
      filter[device].cprev = filter[device].c;

      // Reverse convert the floating point filtered capacitance back into 3 bytes of raw value
      c  = (filter[device].c + 4.096) * (1 << 24) / 8.192;
      ci = (uint32_t) c;

      // Assign back to the messaging buffer
      spiMessageOut.msg.sensor[device].filtCapHigh = (ci >> 16) & 0xFF;
      spiMessageOut.msg.sensor[device].filtCapMid  = (ci >>  8) & 0xFF;
      spiMessageOut.msg.sensor[device].filtCapLow  = (ci      ) & 0xFF;
      break;

    // Single C1 value
    case adcsC1D0:
      spiMessageOut.msg.sensor[device].c1High = rxBuffer[0];
      spiMessageOut.msg.sensor[device].c1Mid  = rxBuffer[1];
      spiMessageOut.msg.sensor[device].c1Low  = rxBuffer[2];
      break;

    // Single C2 value:
    case adcsC2D0:
      spiMessageOut.msg.sensor[device].c2High = rxBuffer[0];
      spiMessageOut.msg.sensor[device].c2Mid  = rxBuffer[1];
      spiMessageOut.msg.sensor[device].c2Low  = rxBuffer[2];
      break;
  }

  // Put the temperature values in each time, even if they're stale
  spiMessageOut.msg.sensor[device].chiptempHigh = rxBuffer[3];
  spiMessageOut.msg.sensor[device].chiptempMid  = rxBuffer[4];
  spiMessageOut.msg.sensor[device].chiptempLow  = rxBuffer[5];

  /* Unlock resource */
  Semaphore_post(semHandle);

  return 0;
}


#endif
























/* -----------------------------------------------------------------------------
 * Sensor RTOS task main loop.  This task handles the communication to each
 * capacitance sensor and the temperature/humidity sensors.
 *
 * Runs and never returns.
 */
static void Sensor_Task(void *pvParameters) {

    portTickType ui32WakeTime;
    uint32_t sensorTaskDelay;
    int32_t heartbeatTimer;

    /* Invoke task every 10ms */
    sensorTaskDelay = 10;
    bool toggle;

    uint8_t vals[6];

    // Get the current tick count.
    ui32WakeTime = xTaskGetTickCount();

    // Count down 500ms, 10ms at a time
    heartbeatTimer = 50;

    // Loop forever.
    while (1) {

        /* This will be invoked once per millisecond */
        heartbeatTimer--;

        if (heartbeatTimer < 0) {

            /*
            I2CReceive(sensor_io[SENSOR1].periph_base, AD7746_ADDR, 0xE3, vals, 6);
            I2CReceive(sensor_io[SENSOR2].periph_base, AD7746_ADDR, 0xE3, vals, 6);
            I2CReceive(sensor_io[SENSOR3].periph_base, AD7746_ADDR, 0xE3, vals, 6);
            I2CReceive(sensor_io[SENSOR4].periph_base, AD7746_ADDR, 0xE3, vals, 6);
            I2CReceive(sensor_io[SENSOR5].periph_base, AD7746_ADDR, 0xE3, vals, 6);
            */
            //I2CInit(SENSOR6);
            I2CReceive(sensor_io[SENSOR6].periph_base, AD7746_ADDR, AD7746_STATUS_REG, vals, 6);


            if (toggle) {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0); // on
            } else {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0); // off
                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 0); // off
            }

            toggle = !toggle;

            heartbeatTimer = 50;
        }

        // Wait for the required amount of time.
        vTaskDelayUntil(&ui32WakeTime, sensorTaskDelay / portTICK_RATE_MS);
    }
}

/* -----------------------------------------------------------------------------
 * Sensor RTOS task initialization, runs once at startup.
 */
uint32_t Sensor_Task_Init(void) {

    /* Initialize the I2C bus for each sensor */
    I2CInit(SENSOR1);
    I2CInit(SENSOR2);
    I2CInit(SENSOR3);
    I2CInit(SENSOR4);
    I2CInit(SENSOR5);
    I2CInit(SENSOR6);

    if(xTaskCreate(Sensor_Task, (const portCHAR *)"SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}
