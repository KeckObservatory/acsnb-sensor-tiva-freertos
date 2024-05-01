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

        /* [1] Status of the single ended sensing */
        uint8_t single_ended_enabled;

        /* [2] State of the relay */
        uint8_t relay_state;

        /* [3] Status of the temp/humidity sensing device */
        uint8_t th_connected;

        /* [4,5] Temperature */
        uint8_t temp_high;
        uint8_t temp_low;

        /* [6,7] Humidity */
        uint8_t humidity_high;
        uint8_t humidity_low;

        /* [8,9,10] Differential capacitance, a 24 bit value */
        uint8_t diff_cap_high;
        uint8_t diff_cap_mid;
        uint8_t diff_cap_low;

        /* [11,12,13] C1 cap single capacitance */
        uint8_t c1_high;
        uint8_t c1_mid;
        uint8_t c1_low;

        /* [14,15,16] C2 cap single capacitance */
        uint8_t c2_high;
        uint8_t c2_mid;
        uint8_t c2_low;

        /* [17,18,19] On-chip temperature from the capacitance sensor */
        uint8_t chip_temp_high;
        uint8_t chip_temp_mid;
        uint8_t chip_temp_low;

    } sensor[MAX_SENSORS];

} __attribute__((packed)) tx_message_t;

/* Message is 5+4+(6*18) = 117 bytes long */
#define SSI_MESSAGE_LENGTH sizeof(tx_message_t)

/* Wrap the message with an array of bytes.  Make 3 copies of this:
 *
 * tx_message_raw is used to create the outbound message contents across various threads
 * tx_message_out is primed with a full checksummed message ready to go out
 * tx_message_dma is a copy of the checksummed message provided to the DMA engine  */
EXTERN union {
    uint8_t      buf[SSI_MESSAGE_LENGTH];
    tx_message_t msg;
} tx_message_raw, tx_message_out, tx_message_dma;

#define tx_message_raw_p (&tx_message_raw.buf[0])
#define tx_message_out_p (&tx_message_out.buf[0])
#define tx_message_dma_p (&tx_message_dma.buf[0])

/* Message from the Beaglebone to the TIVA */
typedef struct {

    /* 5 bytes of Header information first */
    uint8_t checksum;
    uint8_t size;

    /* A pattern that must be provided by the sender; 0xDEADBEEF */
    uint32_t key;

    /* Sensor enable settings (booleans; 0 = disable, 1 = enable) */
    uint8_t enable_sensor1;
    uint8_t enable_sensor2;
    uint8_t enable_sensor3;
    uint8_t enable_sensor4;
    uint8_t enable_sensor5;
    uint8_t enable_sensor6;

    /* Sensor single ended mode settings (booleans; 0 = disable, 1 = enable) */
    uint8_t enable_single1;
    uint8_t enable_single2;
    uint8_t enable_single3;
    uint8_t enable_single4;
    uint8_t enable_single5;
    uint8_t enable_single6;

    /* Relay settings (relay_position_t; 0 = LBL, 1 = Kona) */
    uint8_t relay1;
    uint8_t relay2;
    uint8_t relay3;
    uint8_t relay4;
    uint8_t relay5;
    uint8_t relay6;

} __attribute__((packed)) rx_message_t;

/* Wrap the message with an array of bytes sized to equate to the transmit message,
 * as the received bytes will be the same length as the sent */
EXTERN union {
    uint8_t      buf[SSI_MESSAGE_LENGTH];
    rx_message_t msg;
} rx_message, rx_message_in;

#define rx_message_p (&rx_message.buf[0])
#define rx_message_in_p (&rx_message_in.buf[0])

/* This key pattern must be supplied in messages from the Beaglebone on the SPI for it
 * to be considered valid.  This prevents messages that are all 0x00 from causing changes
 * to the settings. */
#define RX_MESSAGE_KEY 0xDEADBEEF

EXTERN bool rxtx_message_ready;


/* -----------------------------------------------------------------------------
 * Function prototypes
 */
EXTERN void SSI_ISR(void);
EXTERN uint32_t SSI_Task_Init(void);


#undef EXTERN

#endif /* SSI_TASK_H_ */
