/*
 * ssi_task.h - Prototypes for the SSI (SPI) interface task.
 *
 * Copyright (c) 2024, W. M. Keck Observatory
 * All rights reserved.
 *
 * Author: Paul Richards
 *
 */

#ifndef SSI_TASK_H_
#define SSI_TASK_H_

/* Only instantiate variables if we are the .c routine for this header file. */
#ifndef SSI_TASK_C_
  #define EXTERN extern
#else
  #define EXTERN
#endif

#include "includes.h"

/* The control table used by the uDMA controller.  This table must be aligned
   to a 1024 byte boundary. */
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];


/* -----------------------------------------------------------------------------
 * SSI messaging
 *
 * Define a structure which represents the data going back on the SSI, contains
 * all the values of capacitance and temperature and humidity.  Packing
 * is used here to prevent any padding that might be inserted by the compiler.
 *
 * The structure used here closely mimics that used in the ACS motion controller
 * firmware (Cypress).
 * -----------------------------------------------------------------------------
 */

/* Message from the TIVA up to the Beaglebone */
typedef struct {

    /* 5 bytes of Header information first */
    uint8_t checksum;
    uint8_t size;
    uint8_t version0;
    uint8_t version1;
    uint8_t version2;

    /* 4 bytes of the current RTOS tick count that equates to a heartbeat */
    uint32_t tick_count;

    struct {
        /* [0] Status of the AD7746 sensor */
        uint8_t sensor_connected;

        /* [1] Status of the temp/humidity sensing device */
        uint8_t th_connected;

        /* [2,3] Temperature */
        uint8_t temp_high;
        uint8_t temp_low;

        /* [4,5] Humidity */
        uint8_t humidity_high;
        uint8_t humidity_low;

        /* [6,7,8] Differential capacitance, a 24 bit value */
        uint8_t diff_cap_high;
        uint8_t diff_cap_mid;
        uint8_t diff_cap_low;

        /* [9,10,11] C1 cap single capacitance */
        uint8_t c1_high;
        uint8_t c1_mid;
        uint8_t c1_low;

        /* [12,13,14] C2 cap single capacitance */
        uint8_t c2_high;
        uint8_t c2_mid;
        uint8_t c2_low;

        /* [15,16,17] On-chip temperature from the capacitance sensor */
        uint8_t chip_temp_high;
        uint8_t chip_temp_mid;
        uint8_t chip_temp_low;

    } sensor[MAX_SENSORS];

} __attribute__((packed)) tx_message_t;

/* Message is 5+4+(6*18) = 117 bytes long */
#define SSI_MESSAGE_LENGTH sizeof(tx_message_t)

/* Wrap the message with an array of bytes */
EXTERN union {
    uint8_t      buf[SSI_MESSAGE_LENGTH];
    tx_message_t msg;
} tx_message, tx_message_out;

/* TODO: this is a bit of a hack to double buffer the message so the DMA can
 * send it without risk of it being changed out from underneath */
EXTERN uint8_t tx_message_out_dmabuf[SSI_MESSAGE_LENGTH];

/* Message from the Beaglebone to the TIVA */
typedef struct {

    /* First 4 bytes are used for commanding the device */
    uint8_t cmd0;
    uint8_t cmd1;
    uint8_t cmd2;
    uint8_t cmd3;

    /* Subsequent bytes are for settings that are broadcast every messaging cycle */
    uint8_t use_fast_conversion_time;

} __attribute__((packed)) rx_message_t;

/* Wrap the message with an array of bytes sized to equate to the transmit message,
 * as the received bytes will be the same length as the sent */
EXTERN union {
    uint8_t      buf[SSI_MESSAGE_LENGTH];
    rx_message_t msg;
} rx_message, rx_message_in;

EXTERN bool rxtx_message_ready;



/* -----------------------------------------------------------------------------
 * Function prototypes
 * -----------------------------------------------------------------------------
 */
EXTERN void SSI_ISR(void);
EXTERN uint32_t SSI_Task_Init(void);


#undef EXTERN

#endif /* SSI_TASK_H_ */
