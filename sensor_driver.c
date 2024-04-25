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

/* -----------------------------------------------------------------------------
 * This function returns an indication of whether or not the I2C bus has timed
 * out.  The I2C Master Timeout Value must be set.
 */
bool I2C_Master_Timeout(uint32_t ui32Base) {

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
void I2C_Init(sensor_name_t sensor) {

    uint8_t i;
    volatile uint32_t loop;

    uint32_t peripheral = sensor_io[sensor].peripheral;
    uint32_t periph_base = sensor_io[sensor].periph_base;
    uint32_t port_base = sensor_io[sensor].port_base;
    uint32_t scl = sensor_io[sensor].scl;
    uint32_t scl_pin = sensor_io[sensor].scl_pin;
    uint32_t sda = sensor_io[sensor].sda;
    uint32_t sda_pin = sensor_io[sensor].sda_pin;

    /* Dear reader, there is a crazy hack here.
     *
     * It is possible that we interrupted communication with an I2C device.  This will result in a
     * missing stop condition, which leaves the SDA line of the bus in a state (logic 0) where any
     * further attempts to communicate will result in I2C_MASTER_ERR_ARB_LOST and no way to reset
     * out of it.  The SysCtlPeripheralReset() call does not clear the condition.  There is nothing
     * short of power cycling the sensor that would clear the condition.  Until I found this thread:
     *
     *   https://e2e.ti.com/support/wireless-connectivity/wi-fi-group/wifi/f/wi-fi-forum/815551/cc3220sf-i2c-i2c_master_err_arb_lost-on-a-few-boards
     *
     * The recommendation is to switch SCL pin to GPIO, generate a few pulses at SCL line by toggling
     * hi-low state of GPIO output, switch pin back to SCL, then reinitialize the I2C bus.
     *
     * That is what this piece of code below does.  It was verified to work at least once on 2024-02-28
     * when the I2C bus got hung up with SDA in a logic low state.  It took 5 cycles of the clock bus
     * to get SDA back into the right state, so we use 10 cycles to be sure.
     *
     * To use this effectively, detect the ARB lost condition with a call to I2CMasterErr() and re-run
     * the I2CInit() function.
     *
     * This can and will eventually occur when a sensor is hot plugged!
     */


    /* Disable the I2C bus, we're about to rip its I/O lines away */
    SysCtlPeripheralDisable(peripheral);

    /* Set the clock pin up as an output */
    GPIOPinTypeGPIOOutput(port_base, scl_pin);

    /* Strobe 10 clock cycles out to it */
    for (i = 0; i < 10; i++) {
        GPIOPinWrite(port_base, scl_pin, 0);
        for (loop = 0; loop < 5000; loop++) {} // 750us
        GPIOPinWrite(port_base, scl_pin, 1);
        for (loop = 0; loop < 5000; loop++) {} // 750us
    }

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

    /* Enable and initialize the I2C0 master module.  Use the system clock for
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
}


/* -----------------------------------------------------------------------------
 * Send a buffer via I2C to a sensor.
 */
int8_t I2C_Send(uint32_t base, uint32_t slave_addr, uint8_t *buf, uint8_t len) {

    uint8_t i;
    uint32_t err;

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    /* Specify that we are writing to the slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, false);

    /* One byte transmit or multiple? */
    if (len == 1) {

        /* Send the first byte */
        I2CMasterDataPut(base, buf[0]);

        /* Single byte send has a special control mode */
        I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);

        /* Wait for MCU to finish transaction */
        while(I2CMasterBusy(base));

    } else {

        /* Send the first byte */
        I2CMasterDataPut(base, buf[0]);

        /* Start a multi-byte write to the device */
        I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);

        /* Wait for MCU to finish transaction */
        while(I2CMasterBusy(base));

        /* Send the buffer */
        for (i = 1; i < len; i++) {

            I2CMasterDataPut(base, buf[i]);

            /* Last byte finishes the transaction */
            if (i == (len-1)) {
                I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
            } else {
                I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
            }

            /* Wait for MCU to finish transaction */
            while(I2CMasterBusy(base));
        }
    }

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    return 0;
}

/* -----------------------------------------------------------------------------
 * Receive a buffer via I2C from a sensor.
 */
int8_t I2C_Receive_Register(uint32_t base, uint32_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len) {

    uint8_t i;
    uint32_t err;
    bool timeout;

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    /* Specify that we are writing (a register address) to the slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, false);

    /* Specify register to be read */
    I2CMasterDataPut(base, reg);

    /* Send register address byte to slave device */
    I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);

    /* Wait for MCU to finish transaction */
    while(I2CMasterBusy(base));

    /* Now specify that we are going to read from slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, true);

    /* Single byte reads are special */
    if (len == 1) {

        /* Receive just one byte */
        I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);

        /* Wait for MCU to finish transaction */
        while (I2CMasterBusy(base));

        /* Capture one byte */
        buf[0] = I2CMasterDataGet(base);

    } else {
        /* Any length more than 1 is a burst read */

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
            while (I2CMasterBusy(base));

            /* Capture one byte */
            buf[i] = I2CMasterDataGet(base);
        }
    }

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    /* Check for timeout */
    timeout = I2C_Master_Timeout(base);
    if (timeout) {
//        return -5;
    }

    return 0;

}


/* -----------------------------------------------------------------------------
 * Receive a buffer via I2C from a sensor.
 */
int8_t I2C_Receive(uint32_t base, uint32_t slave_addr, uint8_t *buf, uint8_t len) {

    uint8_t i;
    uint32_t err;
    bool timeout;

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    /* Specify that we are going to read from slave device */
    I2CMasterSlaveAddrSet(base, slave_addr, true);

    /* Single byte reads are special */
    if (len == 1) {

        /* Receive just one byte */
        I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);

        /* Wait for MCU to finish transaction */
        while (I2CMasterBusy(base));

        /* Capture one byte */
        buf[0] = I2CMasterDataGet(base);

    } else {
        /* Any length more than 1 is a burst read */

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
            while (I2CMasterBusy(base));

            /* Capture one byte */
            buf[i] = I2CMasterDataGet(base);
        }
    }

    /* Check the I2C bus for errors */
    err = I2CMasterErr(base);

    if (err != I2C_MASTER_ERR_NONE) {

        /* Wait for MCU to time out */
        //while(I2CMasterBusy(base) && !I2CMasterTimeout(base));

        if (err & I2C_MASTER_ERR_ADDR_ACK)
            return -1;
        if (err & I2C_MASTER_ERR_DATA_ACK)
            return -2;
        if (err & I2C_MASTER_ERR_ARB_LOST)
            return -3;
        if (err & I2C_MASTER_ERR_CLK_TOUT)
            return -4;
    }

    /* Check for timeout */
    timeout = I2C_Master_Timeout(base);
    if (timeout) {
//        return -5;
    }

    return 0;

}
