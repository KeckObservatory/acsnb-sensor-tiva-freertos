/*
 * spi_task.h - Prototypes for the SPI interface task.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SPI_TASK_H_
#define SPI_TASK_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SENSOR_TASK_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

/* The control table used by the uDMA controller.  This table must be aligned
   to a 1024 byte boundary. */
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];

// Firmware revision as of 2024-02-20 (PMR)
#define FIRMWARE_REV_0 1
#define FIRMWARE_REV_1 0
#define FIRMWARE_REV_2 0

#define MAX_SENSORS 6


/* -----------------------------------------------------------------------------
 * SPI messaging
 *
 * Define a structure which represents the data going back on the SPI, contains
 * all the values of capacitance and temperature and humidity.  Packing
 * is used here to prevent any padding that might be inserted by the compiler.
 *
 * The structure used here closely mimics that used in the ACS motion controller
 * firmware (Cypress).
 * -----------------------------------------------------------------------------
 */

// Message from the Beaglebone to the TIVA
typedef struct {

    /* First 4 bytes are used for commanding the device */
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t cmd2;
    uint8_t cmd3;

    /* Subsequent bytes are for settings that are broadcast every messaging cycle */
    uint8_t useFastConversionTime;

} __attribute__((packed)) rxMessage_t;

/* Wrap the message with an array of bytes */
EXTERN union {
    uint8_t     buf[5 + (MAX_SENSORS * 19)];
    rxMessage_t msg;
} rxMessage;

#define RX_MESSAGE_LENGTH sizeof(rxMessage)

// Message from the TIVA up to the Beaglebone
typedef struct {

    // 5 bytes of Header information first
    uint8_t checksum;
    uint8_t version0;
    uint8_t version1;
    uint8_t version2;
    uint8_t size;

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

} __attribute__((packed)) txMessage_t;

/* Wrap the message with an array of bytes */
EXTERN union {
    uint8_t     buf[5 + (MAX_SENSORS * 19)];
    txMessage_t msg;
} txMessage;

#define TX_MESSAGE_LENGTH sizeof(txMessage)


EXTERN bool rxMessageReady;
EXTERN bool txMessageReady;



/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */
EXTERN void SPI_ISR(void);
EXTERN uint32_t SPI_Task_Init(void);


#undef EXTERN

#endif /* SPI_TASK_H_ */
