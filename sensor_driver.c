/*
 * sensor_driver.c
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#define SENSOR_DRIVER_C_
#include "includes.h"

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





