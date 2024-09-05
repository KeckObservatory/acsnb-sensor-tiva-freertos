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

/* -----------------------------------------------------------------------------
 * Initialize the PCA9536 relay switching driver.
 */
bool Relay_Init(sensor_name_t sensor) {

    int8_t result;
    uint8_t buf[2] = {0, 0};
    uint32_t base = sensor_io[sensor].periph_base;

    /* Configure the relay switch */
    buf[0] = PCA9536_OUT_PORT_REG;
    buf[1] = PCA9536_OUT_PORT_RESET;
    result = I2C_Send(base, PCA9536_ADDR, buf, 2);
    if (result < 0) return false;

    /* Set all ports as outputs */
    buf[0] = PCA9536_CONFIG_REG;
    buf[1] = PCA9536_CONFIG_ALL_OUTPUT;
    result = I2C_Send(base, PCA9536_ADDR, buf, 2);
    if (result < 0) return false;

    /* If we got this far, reset was successful */
    return true;
}

/* -----------------------------------------------------------------------------
 * Switch the relays to either LBL or Kona mode.
 */
bool Relay_Set(sensor_name_t sensor, relay_position_t position) {

    int8_t result;
    uint8_t buf[2] = {0, 0};
    uint32_t base = sensor_io[sensor].periph_base;

    buf[0] = PCA9536_OUT_PORT_REG;

    /* Set it to LBL or Kona position */
    switch (position) {
        case RELAY_LBL:
        default:
            buf[1] = PCA9536_OUT_PORT_LBL;
            break;

        case RELAY_KONA:
            buf[1] = PCA9536_OUT_PORT_KONA;
            break;
    }

    /* Send the command */
    result = I2C_Send(base, PCA9536_ADDR, buf, 2);
    if (result < 0) return false;

    /* Hold the relay state for 100ms before resetting the driver */
    vTaskDelay(100);

    buf[1] = PCA9536_OUT_PORT_RESET;
    result = I2C_Send(base, PCA9536_ADDR, buf, 2);
    if (result < 0) return false;

    /* If we got this far, setting the relay was successful */
    return true;
}


/* -----------------------------------------------------------------------------
 * Update the positions of the capacitance test set relays.  This is done using
 * a pulse for 4ms on a given line of the MAX7310 device.
 */
bool Cap_Testset_Relays_Set(sensor) {

    int8_t result;
    uint8_t outputs = 0;
    uint8_t buf[2] = {0, 0};
    uint32_t base = sensor_io[sensor].periph_base;


    /* If the device needs initialization, do that now */
    if (!sensor_control[sensor].max7310_configured) {

#ifdef TBD
        /* Disable the timeout feature */
        buf[0] = MAX7310_TIMEOUT_REG;
        buf[1] = MAX7310_TIMEOUT_DISABLE;

        result = I2C_Send(base, MAX7310_ADDR_WRITE, buf, 2);
        if (result < 0) return false;
#endif

        /* Set the output configuration via the output register */
        buf[0] = MAX7310_CFG_REG;
        buf[1] = MAX7310_CFG_SET_OUTPUTS;

        result = I2C_Send(base, MAX7310_ADDR_WRITE, buf, 2);
        if (result < 0) return false;

        sensor_control[sensor].max7310_configured = true;
    }

    /* Set the output states via the output register */
    buf[0] = MAX7310_OUTPUT_REG;

    /* Check U4 first. If U5 and U8 need switching, we will do that the next time this function
     * is called. */
    if (sensor_control[sensor].relay_u4_position != sensor_control[sensor].relay_u4_position_previous) {
        switch (sensor_control[sensor].relay_u4_position) {
            case CAP_RELAY_SET_67_32:
                outputs = CAP_RELAY_U4_67_32;
                break;
            case CAP_RELAY_SET_65_34:
                outputs = CAP_RELAY_U4_65_34;
                break;
        }
        /* Cache the previous value */
        sensor_control[sensor].relay_u4_position_previous = sensor_control[sensor].relay_u4_position;

    } else if (sensor_control[sensor].relay_u5_position != sensor_control[sensor].relay_u5_position_previous) {
        switch (sensor_control[sensor].relay_u5_position) {
            case CAP_RELAY_SET_67_32:
                outputs = CAP_RELAY_U5_67_32;
                break;
            case CAP_RELAY_SET_65_34:
                outputs = CAP_RELAY_U5_65_34;
                break;
        }
        /* Cache the previous value */
        sensor_control[sensor].relay_u5_position_previous = sensor_control[sensor].relay_u5_position;

    } else if (sensor_control[sensor].relay_u8_position != sensor_control[sensor].relay_u8_position_previous) {
        switch (sensor_control[sensor].relay_u8_position) {
            case CAP_RELAY_SET_67_32:
                outputs = CAP_RELAY_U8_67_32;
                break;
            case CAP_RELAY_SET_65_34:
                outputs = CAP_RELAY_U8_65_34;
                break;
        }
        /* Cache the previous value */
        sensor_control[sensor].relay_u8_position_previous = sensor_control[sensor].relay_u8_position;
    }


    /* If there is an output change, send it */
    if (outputs != 0) {

        /* Send the command */
        buf[1] = outputs;
        result = I2C_Send(base, MAX7310_ADDR_WRITE, buf, 2);
        if (result < 0) return false;

        /* Hold the relay state for 4ms before resetting the driver */
        vTaskDelay(MAX7310_HOLD_TIME);

        buf[1] = CAP_RELAY_RESET;
        result = I2C_Send(base, MAX7310_ADDR_WRITE, buf, 2);
        if (result < 0) return false;
    }

    /* If we got this far, everything went as expected */
    return true;


}

/* -----------------------------------------------------------------------------
 * Read the 24AA02UID identity device serial number
 */
bool Identity_Read(sensor) {

    int8_t result;
    uint8_t buf[SERIAL_NUMBER_SIZE] = {0, 0, 0, 0, 0, 0};
    uint32_t base = sensor_io[sensor].periph_base;

    /* First part of ESN at 0xFA0F */
    buf[0] = UC24AA02UID_SN_BASE;
    result = I2C_Send(base, UC24AA02UID_OP_WRITE, buf, 1);
    if (result < 0) return false;

    /* Read 6 bytes */
    result = I2C_Receive(base, UC24AA02UID_OP_READ, buf, SERIAL_NUMBER_SIZE);
    if (result < 0) {

        /* Copy a bogus result into the serial number storage */
        memcpy(sensor_control[sensor].serial_number, serial_number_invalid, SERIAL_NUMBER_SIZE);
        memcpy(tx_message_raw.msg.sensor[sensor].serial_number, serial_number_invalid, SERIAL_NUMBER_SIZE);
        return false;
    } else {

        /* Copy the good result into the serial number storage */
        memcpy(sensor_control[sensor].serial_number, buf, SERIAL_NUMBER_SIZE);
        memcpy(tx_message_raw.msg.sensor[sensor].serial_number, buf, SERIAL_NUMBER_SIZE);
        return true;
    }
}


/* -----------------------------------------------------------------------------
 * Initialize the Si7020 temperature+humidity sensor.
 */
bool TH_Sensor_Init(sensor_name_t sensor) {

    int8_t result1, result2, result3, result4;
    uint8_t buf1[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t buf2[6] = {0, 0, 0, 0, 0, 0};
    uint32_t base = sensor_io[sensor].periph_base;

    /* Determine if a Si7020 device is present.  It may not be connected.  Not
     * all segments will have temperature+humidity sensing.  Thus, try to
     * read the electronic serial number (ESN) from it.  This occurs in two
     * reads.  The first is 8 bytes, the second is 6, at differing addresses. */

    /* First part of ESN at 0xFA0F */
    buf1[0] = SI7020_READ_ESN1_1;
    buf1[1] = SI7020_READ_ESN1_2;
    result1 = I2C_Send(base, SI7020_ADDR, buf1, 2);

    /* Read 8 bytes */
    result2 = I2C_Receive(base, SI7020_ADDR, buf1, 8);

    /* Second part of ESN at 0xFCC9 */
    buf2[0] = SI7020_READ_ESN2_1;
    buf2[1] = SI7020_READ_ESN2_2;
    result3 = I2C_Send(base, SI7020_ADDR, buf2, 2);

    /* Read 6 bytes */
    result4 = I2C_Receive(base, SI7020_ADDR, buf2, 6);

    /* If all 4 transactions completed, we can trust the results */
    if ((result1 + result2 + result3 + result4) == 0) {

        /* Assemble the ESN from the piece parts in the buffers */
        sensor_control[sensor].si7020_esn[0] = buf1[SI7020_SNA_0];
        sensor_control[sensor].si7020_esn[1] = buf1[SI7020_SNA_1];
        sensor_control[sensor].si7020_esn[2] = buf1[SI7020_SNA_2];
        sensor_control[sensor].si7020_esn[3] = buf1[SI7020_SNA_3];
        sensor_control[sensor].si7020_esn[4] = buf2[SI7020_SNB_0];
        sensor_control[sensor].si7020_esn[5] = buf2[SI7020_SNB_1];
        sensor_control[sensor].si7020_esn[6] = buf2[SI7020_SNB_2];
        sensor_control[sensor].si7020_esn[7] = buf2[SI7020_SNB_3];

        /* Detect if it's an Si7020 or an Si7021, either is OK */
        if ((buf2[SI7020_SNB_3] == SI7020_ID) || (buf2[SI7020_SNB_3] == SI7021_ID)) {
            sensor_control[sensor].si7020_connected = true;
        } else {
            sensor_control[sensor].si7020_connected = false;
            return false;
        }

    } else {
        sensor_control[sensor].si7020_connected = false;
        return false;
    }

    /* If we got this far, init was successful */
    return true;
}

/* -----------------------------------------------------------------------------
 * Read the Si7020 temperature+humidity sensor.
 */
bool TH_Sensor_Read(sensor_name_t sensor) {

    int8_t result1, result2, result3, result4;
    uint8_t buf_t[2], buf_h[2];
    uint32_t base = sensor_io[sensor].periph_base;


    /* Read temperature */
    buf_t[0] = SI7020_TMP_HOLD;
    buf_t[1] = 0;
    result1 = I2C_Send(base, SI7020_ADDR, buf_t, 1);
    result2 = I2C_Receive(base, SI7020_ADDR, buf_t, 2);

    /* Read humidity */
    buf_h[0] = SI7020_HUM_HOLD;
    buf_h[1] = 0;
    result3 = I2C_Send(base, SI7020_ADDR, buf_h, 1);
    result4 = I2C_Receive(base, SI7020_ADDR, buf_h, 2);

    /* If all 4 transactions completed, we can trust the results */
    if ((result1 + result2 + result3 + result4) == 0) {

        /* Lock the structure with the message, by taking the semaphore */
        if (xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY) == pdTRUE) {

            /* We got the lock and can now work with the message exclusively
             * to update the outgoing message with the new values */

            tx_message_raw.msg.sensor[sensor].temp_high     = buf_t[0];
            tx_message_raw.msg.sensor[sensor].temp_low      = buf_t[1];
            tx_message_raw.msg.sensor[sensor].humidity_high = buf_h[0];
            tx_message_raw.msg.sensor[sensor].humidity_low  = buf_h[1];

            /* Release the semaphore */
            xSemaphoreGive(g_txMessageSemaphore);
        }

    } else {

        /* Lock the structure with the message, by taking the semaphore */
        if (xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY) == pdTRUE) {

            /* We got the lock and can now work with the message exclusively
             * to update the outgoing message with the new values */

            tx_message_raw.msg.sensor[sensor].temp_high     = SI7020_INVALID_TH;
            tx_message_raw.msg.sensor[sensor].temp_low      = SI7020_INVALID_TL;
            tx_message_raw.msg.sensor[sensor].humidity_high = SI7020_INVALID_HH;
            tx_message_raw.msg.sensor[sensor].humidity_low  = SI7020_INVALID_HL;

            /* Sensor is now disconnected */
            sensor_control[sensor].si7020_connected = false;


            /* Release the semaphore */
            xSemaphoreGive(g_txMessageSemaphore);
        }

        return false;
    }

    /* If we got this far, read was successful */
    return true;
}


/* -----------------------------------------------------------------------------
 * Reset a capacitance sensor.
 */
bool Sensor_Reset(sensor_name_t sensor) {

    int8_t result;
    uint8_t buf[2];
    uint32_t base = sensor_io[sensor].periph_base;

    /* Configure capacitance measurement to default (differential) */
    buf[0] = AD7746_RESET_REG;
    result = I2C_Send(base, AD7746_ADDR, buf, 1);
    if (result < 0) return false;

    /* If we got this far, reset was successful */
    return true;
}


/* -----------------------------------------------------------------------------
 * Initialize a capacitance sensor.
 */
bool Sensor_Init(sensor_name_t sensor) {

    int8_t result;
    uint8_t buf[2];
    uint32_t base = sensor_io[sensor].periph_base;

    /* Read the status register to verify the device is there */
    result = I2C_Receive_Register(base, AD7746_ADDR, AD7746_READ, buf, 1);
    if (result < 0) return false;

    /* Configure capacitance measurement to default (differential) */
    buf[0] = AD7746_CAP_SETUP_REG;
    buf[1] = AD7746_CAP_DIFFERENTIAL;
    result = I2C_Send(base, AD7746_ADDR, buf, 2);
    if (result < 0) return false;

    /* Configure voltage/temperature (enable internal temperature sensor) */
    buf[0] = AD7746_VT_SETUP_REG;
    buf[1] = AD7746_VT_SETUP_INT_TEMP;
    result = I2C_Send(base, AD7746_ADDR, buf, 2);
    if (result < 0) return false;

    /* Configure excitation */
    buf[0] = AD7746_EXC_SETUP_REG;
    buf[1] = AD7746_EXC_SET_A;
    result = I2C_Send(base, AD7746_ADDR, buf, 2);
    if (result < 0) return false;

    /* If we got this far, init was successful */
    return true;
}


/* -----------------------------------------------------------------------------
 * Trigger a capacitance sensor conversion.
 */
bool Sensor_Trigger(sensor_name_t sensor, sensor_mode_t cap_mode) {

    int8_t result;
    uint8_t buf[2];
    uint32_t base = sensor_io[sensor].periph_base;

    /* Configure capacitance or temperature measurement */
    switch (cap_mode) {

        case MODE_C_DIFFERENTIAL:
        case MODE_C_CAP1:
        case MODE_C_CAP2:
            /* Configure which capacitor to sample */
            buf[0] = AD7746_CAP_SETUP_REG;

            /* Differential will be the mode, almost always! */
            buf[1] = AD7746_CAP_DIFFERENTIAL;

            /* But check for alternate modes used during testing... */
            if (cap_mode == MODE_C_CAP1) {
                buf[1] = AD7746_CAP_CAP1;
            } else if (cap_mode == MODE_C_CAP2) {
                buf[1] = AD7746_CAP_CAP2;
            }

            result = I2C_Send(base, AD7746_ADDR, buf, 2);
            if (result < 0) return false;

            /* Configure capacitance timing */
            buf[0] = AD7746_CFG_REG;
            switch (sensor_control[sensor].conversion_time) {

                case CONVERT_TIME_38MS:
                    buf[1] = AD7746_CFG_C_38MS_SINGLE;
                    break;

                case CONVERT_TIME_11MS:
                    buf[1] = AD7746_CFG_C_11MS_SINGLE;
                    break;

                case CONVERT_TIME_109MS:
                default:
                    buf[1] = AD7746_CFG_C_109MS_SINGLE;
                    break;
            }
            result = I2C_Send(base, AD7746_ADDR, buf, 2);
            if (result < 0) return false;

            break;

        case MODE_TEMPERATURE:

            /* Build message to device: set conversion time and trigger conversion */
            buf[0] = AD7746_CFG_REG;
            buf[1] = AD7746_CFG_T_DEFAULT; /* Default to 32ms temperature conversion time */
            result = I2C_Send(base, AD7746_ADDR, buf, 2);
            if (result < 0) return false;

            break;

        default:
            /* There is no default conversion, something got messed up! */
            return false;
    }

    return true;
}


/* -----------------------------------------------------------------------------
 * Read a capacitance sensor value.
 */
bool Sensor_Read(sensor_name_t sensor, sensor_mode_t cap_mode) {

    int8_t i2c_result;
    uint8_t buf[8];
    uint32_t base = sensor_io[sensor].periph_base;

    /* Read the capacitance and temperature conversion results,
     * 3 bytes each (see spec page 14).  Read it all every time to keep
     * this routine simple! */
    i2c_result = I2C_Receive_Register(base, AD7746_ADDR, AD7746_READ, buf, 7);
    if (i2c_result < 0) return false;

    /* Lock the structure with the message, by taking the semaphore */
    if (xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY) == pdTRUE) {

        /* We got the lock and can now work with the message exclusively
         * to update the outgoing message with the new values */
        tx_message_raw.msg.sensor[sensor].single_ended_enabled = sensor_control[sensor].enable_c1_c2;
        tx_message_raw.msg.sensor[sensor].relay_state = sensor_control[sensor].relay_position;

        /* What read mode resulted in this value? */
        switch(cap_mode) {

          /* Differential capacitor value */
          case MODE_C_DIFFERENTIAL:
              tx_message_raw.msg.sensor[sensor].diff_cap_high = buf[1];
              tx_message_raw.msg.sensor[sensor].diff_cap_mid  = buf[2];
              tx_message_raw.msg.sensor[sensor].diff_cap_low  = buf[3];
              break;

          /* Single C1 value */
          case MODE_C_CAP1:
              tx_message_raw.msg.sensor[sensor].c1_high = buf[1];
              tx_message_raw.msg.sensor[sensor].c1_mid  = buf[2];
              tx_message_raw.msg.sensor[sensor].c1_low  = buf[3];
              break;

          /* Single C2 value */
          case MODE_C_CAP2:
              tx_message_raw.msg.sensor[sensor].c2_high = buf[1];
              tx_message_raw.msg.sensor[sensor].c2_mid  = buf[2];
              tx_message_raw.msg.sensor[sensor].c2_low  = buf[3];
              break;

          /* Temperature conversion */
          case MODE_TEMPERATURE:
              tx_message_raw.msg.sensor[sensor].chip_temp_high = buf[4];
              tx_message_raw.msg.sensor[sensor].chip_temp_mid  = buf[5];
              tx_message_raw.msg.sensor[sensor].chip_temp_low  = buf[6];
              break;

          /* No default applies to this switch block */
          default:
              break;

        }

        /* Release the semaphore */
        xSemaphoreGive(g_txMessageSemaphore);
    }

    /* If we got this far, read was successful */
    return true;
}


/* -----------------------------------------------------------------------------
 * Sensor state machine processing function.  This runs every 10ms.
 */
void Sensor_Process(sensor_name_t sensor) {

#define TO_STATE(s) (*p_state = s)

    bool result = false;

    /* Don't process disabled sensors */
    if (!sensor_control[sensor].enabled) {
        tx_message_raw.msg.sensor[sensor].sensor_connected = false;
        tx_message_raw.msg.sensor[sensor].th_connected = false;
        return;
    }

    /* Get the values used to drive the state machine from the control structure */
    sensor_state_t *p_state               = &(sensor_control[sensor].state);
    sensor_mode_t *p_mode                 = &(sensor_control[sensor].mode);
    sensor_mode_t *p_next_mode            = &(sensor_control[sensor].next_mode);
    sensor_mode_t *p_last_mode            = &(sensor_control[sensor].last_mode);
    uint8_t *p_conversions                = &(sensor_control[sensor].conversions);
    bool enable_c1_c2                     =   sensor_control[sensor].enable_c1_c2;
    bool *p_cap_connected                 = &(sensor_control[sensor].ad7746_connected);
    timer_t *p_timer_init                 = &(sensor_control[sensor].timer_init);
    timer_t *p_timer_ready                = &(sensor_control[sensor].timer_ready);
    bool *p_th_connected                  = &(sensor_control[sensor].si7020_connected);
    timer_t *p_th_timer                   = &(sensor_control[sensor].si7020_timer);
    relay_position_t *p_relay             = &(sensor_control[sensor].relay_position);
    relay_position_t *p_relay_prev        = &(sensor_control[sensor].relay_position_previous);

    /* Support for the capacitance test set */
    bool *p_max7310_connected             = &(sensor_control[sensor].max7310_connected);
    bool *p_max7310_configured            = &(sensor_control[sensor].max7310_configured);
    cap_relay_position_t *p_u4_relay      = &(sensor_control[sensor].relay_u4_position);
    cap_relay_position_t *p_u4_relay_prev = &(sensor_control[sensor].relay_u4_position_previous);
    cap_relay_position_t *p_u5_relay      = &(sensor_control[sensor].relay_u5_position);
    cap_relay_position_t *p_u5_relay_prev = &(sensor_control[sensor].relay_u5_position_previous);
    cap_relay_position_t *p_u8_relay      = &(sensor_control[sensor].relay_u8_position);
    cap_relay_position_t *p_u8_relay_prev = &(sensor_control[sensor].relay_u8_position_previous);

    /* Reference the ready flag for this sensor; note that this is a pointer already in the struct! */
    bool *p_ready_flag             = sensor_io[sensor].isr_flag;

    /* Update the outbound message fields */
    tx_message_raw.msg.sensor[sensor].sensor_connected = *p_cap_connected;
    tx_message_raw.msg.sensor[sensor].th_connected     = *p_th_connected;


    switch (*p_state) {

        /* Power-on-reset state */
        case STATE_POR:
        default:

            /* Start off disconnected */
            *p_cap_connected = false;
            *p_th_connected = false;

            /* Setup the initialization timer */
            timer_set(p_timer_init, SENSOR_INIT_TIMEOUT_MS);

            /* Expire the init timer so it happens immediately */
            timer_expire(p_timer_init);

            /* Setup the ready signal timer */
            timer_set(p_timer_ready, SENSOR_READY_TIMEOUT_MS);

            /* Setup the temp+humidity sensor timer */
            timer_set(p_th_timer, TH_TIMEOUT_MS);

            TO_STATE(STATE_IDLE);
            break;

        /* Check timers and run subsystems */
        case STATE_IDLE:

            /* If the sensor is connected we can talk on the bus to get cap or do relays */
            if (*p_cap_connected) {

                /* If this is a normal sensor, it will have the PCA9536 relay device.  Else it is
                 * connected to a capacitance sensor test set which uses a MAX7310 */
                if (*p_max7310_connected) {
                    if ((*p_u4_relay != *p_u4_relay_prev) ||
                        (*p_u5_relay != *p_u5_relay_prev) ||
                        (*p_u8_relay != *p_u8_relay_prev)) {

                        /* Relay position demands have changed, update them */
                        Cap_Testset_Relays_Set(sensor);
                    }

                } else {
                    /* If the PCA9536 relay position demand has changed, set them */
                    if (*p_relay != *p_relay_prev) {
                        Relay_Set(sensor, *p_relay);

                        /* Store the new position as previous so we don't end up sending
                         * the commands to the relay over and over */
                        *p_relay_prev = *p_relay;
                    }
                }

                /* Interleave cap conversions with on-chip temperature converts */
                if (*p_conversions < AD7746_TEMP_TRIGGER_RATE) {
                    TO_STATE(STATE_TRIGGER_CAP);
                } else {

                    /* Get the temperature and humidity from the external sensor
                     * on a periodic basis.  If it's not time yet, just read the
                     * on-chip temperature from the AD7746. */
                    if (timer_expired(p_th_timer)) {
                        TO_STATE(STATE_READ_TH);
                    } else {
                        TO_STATE(STATE_TRIGGER_TEMPERATURE);
                    }
                }

            /* Else, try to reset+init the sensor periodically */
            } else if (timer_expired(p_timer_init)) {
                TO_STATE(STATE_RESET);
            }

            break;

        /* Reset the AD7746 device.  Note: this must occur in its own state, in order to give
         * the device a bit of time before triggering the first conversion. */
        case STATE_RESET:
            /* Pre-clear the sensor ready flag */
            *p_ready_flag = false;

            /* Init the I2C bus, this will clear a hung I2C bus from an incomplete transaction */
            I2C_Init(sensor);

            /* Init the AD7746 capacitance sensor: send a 0xBF reset value to the sensor
             * to see if it's there */
            result = Sensor_Reset(sensor);

            /* Is the sensor responding? */
            if (result) {
                /* Mark the sensor as connected */
                *p_cap_connected = true;
                TO_STATE(STATE_INIT);

            /* If the sensor is unplugged the reset will have failed */
            } else {
                /* Mark the sensor as disconnected and return to idle */
                *p_cap_connected = false;
                TO_STATE(STATE_IDLE);
            }

            break;

        /* Initialize the I2C devices (cap sensor, cap relays) */
        case STATE_INIT:

            /* Read the identity device */
            Identity_Read(sensor);

            /* Try to init the sensor */
            if (!Sensor_Init(sensor)) {
                /* Failed, mark the sensor as disconnected before returning to idle  */
                *p_cap_connected = false;
            }

            /* Attempt to init the switching relay.  If that fails, assume the capacitance
             * test set is connected and try using that instead. */
            //if (!Relay_Init(sensor)) {
                *p_max7310_connected = true;
                *p_max7310_configured = false;
            //} else {
            //    *p_max7310_connected = false;
            //}

            /* Always go back to idle so the timers can run */
            TO_STATE(STATE_IDLE);
            break;


        /* Start a new capacitance conversion */
        case STATE_TRIGGER_CAP:

            /* Clear the ready flag for the next read */
            *p_ready_flag = false;

            /* If single ended captures are enabled, then cycle through the three cap types
             * (differential, then c1, then c2) for two full cycles before advancing to do
             * a chip temperature read */
            if (enable_c1_c2) {

                /* Set the next mode based on the last */
                switch (*p_last_mode) {
                    case MODE_C_DIFFERENTIAL:
                    default:
                        *p_next_mode = MODE_C_CAP1;
                        break;

                    case MODE_C_CAP1:
                        *p_next_mode = MODE_C_CAP2;
                        break;

                    case MODE_C_CAP2:
                        *p_next_mode = MODE_C_DIFFERENTIAL;
                        break;
                }

                /* Do the assignment now */
                *p_mode = *p_next_mode;
                *p_last_mode = *p_next_mode;

            } else {

                /* Single ended disabled, so always set the mode to differential capture */
                *p_mode = MODE_C_DIFFERENTIAL;
            }

            /* Tell the device to start conversion */
            result = Sensor_Trigger(sensor, *p_mode);

            if (result) {
                /* Advance to the next state to await the conversion result, or time out */
                TO_STATE(STATE_TRIGGER_CAP_WAIT);
            } else {
                /* Mark the sensor as disconnected and return to idle */
                *p_cap_connected = false;
                TO_STATE(STATE_IDLE);
            }

            /* Start timing the ready signal */
            timer_start(p_timer_ready);

            break;

        /* Await the ready flag */
        case STATE_TRIGGER_CAP_WAIT:

            /* Check for ready signal timeout */
            if (timer_expired(p_timer_ready)) {
                //TODO: count the number of timeouts?
                TO_STATE(STATE_POR);
                return;
            }

            /* Did the conversion complete yet? */
            if (*p_ready_flag) {
                result = Sensor_Read(sensor, *p_mode);

                /* Count how many differential conversions performed, for interleaving
                 * on-chip temperature reads */
                *p_conversions += 1;

                /* Return to idle for next event */
                TO_STATE(STATE_IDLE);
            }

            break;

        /* Start a new temperature conversion */
        case STATE_TRIGGER_TEMPERATURE:

            /* Clear the ready flag for the next read */
            *p_ready_flag = false;

            /* Reset the conversions count */
            *p_conversions = 0;

            /* Set the mode to temperature capture */
            *p_mode = MODE_TEMPERATURE;

            /* Tell the device to start conversion */
            result = Sensor_Trigger(sensor, *p_mode);

            if (result) {
                /* Start timing the ready signal */
                timer_start(p_timer_ready);

                /* Advance to the next state to await the conversion result, or time out */
                TO_STATE(STATE_TRIGGER_TEMP_WAIT);
            } else {
                /* Mark the sensor as disconnected and return to idle */
                *p_cap_connected = false;
                TO_STATE(STATE_IDLE);
            }

            break;

        /* Await the ready flag */
        case STATE_TRIGGER_TEMP_WAIT:

            /* Check for ready signal timeout */
            if (timer_expired(p_timer_ready)) {
                //TODO: count the number of timeouts?
                TO_STATE(STATE_POR);
                return;
            }

            /* Did the conversion complete yet? */
            if (*p_ready_flag) {
                Sensor_Read(sensor, *p_mode);

                /* Return to idle for next event */
                TO_STATE(STATE_IDLE);
            }
            break;

        /* Read the temperature + humidity */
        case STATE_READ_TH:

            /* If a T+H sensor isn't connected, try to detect it and use it */
            if (!(*p_th_connected)) {

                /* Detect the presence of the temperature + humidity sensor */
                TH_Sensor_Init(sensor);
            }

            /* If a T+H sensor is now connected do a t+h read */
            if (*p_th_connected) {
                TH_Sensor_Read(sensor);
            }

            /* Restart the temp+hum reading timer */
            timer_start(p_th_timer);

            /* Return to idle for next event */
            TO_STATE(STATE_IDLE);
            break;

    }
}


/* -----------------------------------------------------------------------------
 * Sensor RTOS task main loop.  This task handles the communication to each
 * capacitance sensor and the temperature/humidity sensors.
 *
 * Runs and never returns.
 */
static void Sensor_Task(void *pvParameters) {

    portTickType wake_time;
    uint32_t sensorTaskDelay;
    sensor_name_t sensor;

    int32_t heartbeatTimer;

    /* Delay 10ms per execution of the loop */
    uint32_t task_delay = 10;

    /* Get the current tick count */
    wake_time = xTaskGetTickCount();

    // Count down 500ms, 10ms at a time
    heartbeatTimer = 50;

    // Loop forever.
    while (1) {

        /* Run the state machine once for each sensor, should take about 1ms each */
        for (sensor = SENSOR1; sensor < MAX_SENSORS; sensor++) {
            Sensor_Process(sensor);
        }


#ifdef zero

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

            heartbeatTimer = 50;
        }
#endif

        /* Wait for the required amount of time */
        //vTaskDelayUntil(&wake_time, task_delay / portTICK_RATE_MS);
        vTaskDelay(task_delay / portTICK_RATE_MS);
    }
}

/* -----------------------------------------------------------------------------
 * Sensor RTOS task initialization, runs once at startup.
 */
uint32_t Sensor_Task_Init(void) {

    sensor_name_t sensor;

    /* Setup the sensor control structure */
    for (sensor = SENSOR1; sensor < MAX_SENSORS; sensor++) {

        /* Initialize the fields to sane defaults */
        sensor_control[sensor].state                      = STATE_POR;
        sensor_control[sensor].enabled                    = false;
        sensor_control[sensor].relay_position             = RELAY_LBL;
        sensor_control[sensor].mode                       = MODE_C_DIFFERENTIAL;
        sensor_control[sensor].next_mode                  = MODE_C_DIFFERENTIAL;
        sensor_control[sensor].last_mode                  = MODE_C_DIFFERENTIAL;
        sensor_control[sensor].conversion_time            = CONVERT_TIME_109MS;
        sensor_control[sensor].enable_c1_c2               = false;
        memcpy(sensor_control[sensor].serial_number, serial_number_default, SERIAL_NUMBER_SIZE);

        /* Defaults for the capacitance test set, initial state is not knowable because the
         * relays will hold their position (through a power cycle!) until commanded to change,
         * so start off in the "connected" state for everything. */
        sensor_control[sensor].max7310_connected          = false;
        sensor_control[sensor].max7310_configured         = false;
        sensor_control[sensor].relay_u4_position          = CAP_RELAY_SET_UNKNOWN;
        sensor_control[sensor].relay_u4_position_previous = CAP_RELAY_SET_UNKNOWN;
        sensor_control[sensor].relay_u5_position          = CAP_RELAY_SET_UNKNOWN;
        sensor_control[sensor].relay_u5_position_previous = CAP_RELAY_SET_UNKNOWN;
        sensor_control[sensor].relay_u8_position          = CAP_RELAY_SET_UNKNOWN;
        sensor_control[sensor].relay_u8_position_previous = CAP_RELAY_SET_UNKNOWN;

        /* Initialize the I2C bus for the sensor */
        I2C_Init(sensor);

        /* Set sane defaults in the messaging for the values, no need to
         * try to lock the message here as it will not be used yet */
        tx_message_raw.msg.sensor[sensor].temp_high       = SI7020_INVALID_TH;
        tx_message_raw.msg.sensor[sensor].temp_low        = SI7020_INVALID_TL;
        tx_message_raw.msg.sensor[sensor].humidity_high   = SI7020_INVALID_HH;
        tx_message_raw.msg.sensor[sensor].humidity_low    = SI7020_INVALID_HL;
        memcpy(tx_message_raw.msg.sensor[sensor].serial_number, serial_number_default, SERIAL_NUMBER_SIZE);
    }


    /* Testing: enabling some sensors */
    sensor_control[SENSOR1].enabled = true;
    sensor_control[SENSOR1].enable_c1_c2 = true;

    if(xTaskCreate(Sensor_Task, (const portCHAR *)"SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}

