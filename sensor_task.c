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
 * Reset a capacitance sensor.
 */
bool Sensor_Reset(sensor_name_t sensor) {

    int8_t result;
    uint8_t buf[2];
    uint32_t base = sensor_io[sensor].periph_base;

    /* Configure capacitance measurement to default (differential) */
    buf[0] = AD7746_RESET_REG;
    result = I2CSend(base, AD7746_ADDR, buf, 1);
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
    result = I2CReceive(base, AD7746_ADDR, AD7746_READ, buf, 1);
    if (result < 0) return false;

    /* Configure capacitance measurement to default (differential) */
    buf[0] = AD7746_CAP_SETUP_REG;
    buf[1] = AD7746_CAP_DIFFERENTIAL;
    result = I2CSend(base, AD7746_ADDR, buf, 2);
    if (result < 0) return false;

    /* Configure voltage/temperature (enable internal temperature sensor) */
    buf[0] = AD7746_VT_SETUP_REG;
    buf[1] = AD7746_VT_SETUP_INT_TEMP;
    result = I2CSend(base, AD7746_ADDR, buf, 2);
    if (result < 0) return false;

    /* Configure excitation */
    buf[0] = AD7746_EXC_SETUP_REG;
    buf[1] = AD7746_EXC_SET_A;
    result = I2CSend(base, AD7746_ADDR, buf, 2);
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

            result = I2CSend(base, AD7746_ADDR, buf, 2);
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
            result = I2CSend(base, AD7746_ADDR, buf, 2);
            if (result < 0) return false;

            break;

        case MODE_TEMPERATURE:

            /* Build message to device: set conversion time and trigger conversion */
            buf[0] = AD7746_CFG_REG;
            buf[1] = AD7746_CFG_T_DEFAULT; /* Default to 32ms temperature conversion time */
            result = I2CSend(base, AD7746_ADDR, buf, 2);
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
    i2c_result = I2CReceive(base, AD7746_ADDR, AD7746_READ, buf, 7);
    if (i2c_result < 0) return false;

    /* Lock the structure with the message, by taking the semaphore */
    if (xSemaphoreTake(g_txMessageSemaphore, portMAX_DELAY) == pdTRUE) {

        /* We got the lock and can now work with the message exclusively
         * to update the outgoing message with the new values */

        /* What read mode resulted in this value? */
        switch(cap_mode) {

          /* Differential capacitor value */
          case MODE_C_DIFFERENTIAL:
              tx_message.msg.sensor[sensor].diff_cap_high = buf[1];
              tx_message.msg.sensor[sensor].diff_cap_mid  = buf[2];
              tx_message.msg.sensor[sensor].diff_cap_low  = buf[3];
              break;

          /* Single C1 value */
          case MODE_C_CAP1:
              tx_message.msg.sensor[sensor].c1_high = buf[1];
              tx_message.msg.sensor[sensor].c1_mid  = buf[2];
              tx_message.msg.sensor[sensor].c1_low  = buf[3];
              break;

          /* Single C2 value */
          case MODE_C_CAP2:
              tx_message.msg.sensor[sensor].c2_high = buf[1];
              tx_message.msg.sensor[sensor].c2_mid  = buf[2];
              tx_message.msg.sensor[sensor].c2_low  = buf[3];
              break;

          /* Temperature conversion */
          case MODE_TEMPERATURE:
              tx_message.msg.sensor[sensor].chip_temp_high = buf[4];
              tx_message.msg.sensor[sensor].chip_temp_mid  = buf[5];
              tx_message.msg.sensor[sensor].chip_temp_low  = buf[6];
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
 * Sensor state machine processing function.
 */
void Sensor_Process(sensor_name_t sensor) {

    bool result = false;
    static uint32_t state_start_tick = 0;
    uint32_t now = 0;
    uint32_t elapsed = 0;
    static sensor_mode_t next_sensor_mode, last_sensor_mode = MODE_C_DIFFERENTIAL;

    /* Get the values used to drive the state machine from the control structure */
    sensor_state_t *p_state = &(sensor_control[sensor].state);
    bool disabled = sensor_control[sensor].disabled;
    sensor_mode_t *p_mode = &(sensor_control[sensor].mode);
    uint8_t *p_conversions = &(sensor_control[sensor].conversions);
    bool enable_c1_c2 = sensor_control[sensor].enable_c1_c2;
    bool *p_connected = &(sensor_control[sensor].connected);
    int32_t *p_init_wait_timer = &(sensor_control[sensor].init_wait_timer);

    /* Reference the ready flag for this sensor; note that this is a pointer already in the struct! */
    bool *p_ready_flag = sensor_io[sensor].isr_flag;

    //sensor_relay_position_t relay_position = sensor_control[sensor].relay_position;

    /* Don't process disabled sensors */
    if (disabled) return;

    /* Get the current time for calculating timeouts */
    now = xTaskGetTickCount();

    switch (*p_state) {

        /* Power-on-reset state; init the device */
        case STATE_POR:
        default:

            /* Re-drive the I2C initialization of the bus */
            I2CInit(sensor);

            /* Send the 0xBF reset value to the sensor to see if it's there */
            result = Sensor_Reset(sensor);

            /* Is the sensor responding? */
            if (result) {
                *p_state = STATE_INIT;
                *p_connected = true;

            /* If the sensor is unplugged the reset will fail */
            } else {

                /* Fault the sensor, which will start waiting for it to be connected */
                *p_state = STATE_FAULTED;
            }

            break;

        /* Initialize the sensor */
        case STATE_INIT:
            /* Pre-clear the sensor ready flag */
            *p_ready_flag = false;

            /* Reset completed, try to init the sensor */
            result = Sensor_Init(sensor);

            if (result) {
                /* Proceed to triggering */
                *p_state = STATE_TRIGGER_CAP;
            } else {
                /* Fault the sensor, which will start waiting for it to be connected */
                *p_state = STATE_FAULTED;
            }

            break;

        /* Wait for a specified amount of time before attempting to reset the sensor
         * and try again */
        case STATE_INIT_WAIT:
            /* How long have we been waiting? */
            elapsed = now - state_start_tick;

            /* Deduct that from the re-loaded timer */
            *p_init_wait_timer -= elapsed;

            /* When timer goes below 0, try to init the sensor again */
            if (*p_init_wait_timer < 0) {
                *p_state = STATE_POR;
            }

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
                switch (last_sensor_mode) {
                    case MODE_C_DIFFERENTIAL:
                    default:
                        next_sensor_mode = MODE_C_CAP1;
                        break;

                    case MODE_C_CAP1:
                        next_sensor_mode = MODE_C_CAP2;
                        break;

                    case MODE_C_CAP2:
                        next_sensor_mode = MODE_C_DIFFERENTIAL;
                        break;
                }

                /* Do the assignment now */
                *p_mode = next_sensor_mode;
                last_sensor_mode = next_sensor_mode;

            } else {

                /* Single ended disabled, so always set the mode to differential capture */
                *p_mode = MODE_C_DIFFERENTIAL;
            }

            /* Tell the device to start conversion */
            result = Sensor_Trigger(sensor, *p_mode);

            if (result) {
                /* Advance to the next state to await the conversion result, or time out */
                *p_state = STATE_TRIGGER_CAP_WAIT;
            } else {
                /* Fault the sensor, which will start waiting for it to be connected */
                *p_state = STATE_FAULTED;
            }

            /* Start timing the wait state */
            state_start_tick = xTaskGetTickCount();

            break;

        /* Await the ready flag */
        case STATE_TRIGGER_CAP_WAIT:

            /* Check for timeout after 1 second */
            elapsed = now - state_start_tick;
            if (elapsed > 1000) {
                *p_state = STATE_POR;
                return;
            }

            /* Did the conversion complete since last invocation of the state machine? */
            if (*p_ready_flag) {
                Sensor_Read(sensor, *p_mode);

                /* Count how many differential conversions */
                *p_conversions += 1;
                if (*p_conversions < AD7746_TEMP_TRIGGER_RATE) {
                    *p_state = STATE_TRIGGER_CAP;
                } else {
                    *p_state = STATE_TRIGGER_TEMPERATURE;
                }
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
                /* Advance to the next state to await the conversion result, or time out */
                *p_state = STATE_TRIGGER_TEMP_WAIT;
            } else {
                /* Fault the sensor, which will start waiting for it to be connected */
                *p_state = STATE_FAULTED;
            }

            /* Start timing the wait state */
            state_start_tick = xTaskGetTickCount();

            break;

        /* Await the ready flag */
        case STATE_TRIGGER_TEMP_WAIT:

            /* Check for timeout after 1 second */
            elapsed = now - state_start_tick;
            if (elapsed > 1000) {
                *p_state = STATE_POR;
                return;
            }

            /* Did the conversion complete since last invocation of the state machine? */
            if (*p_ready_flag) {
                Sensor_Read(sensor, *p_mode);

                /* Return to converting capacitances */
                *p_state = STATE_TRIGGER_CAP;
            }

            break;

        /* Device has faulted, restart the POR init process */
        case STATE_FAULTED:

            /* Mark the sensor as disconnected */
            *p_connected = false;

            /* Start a timer to reconnect in 1 second */
            *p_init_wait_timer = SENSOR_REINIT_TIMEOUT_MS;

            /* Start timing the wait state */
            state_start_tick = xTaskGetTickCount();

            /* Enter the wait state */
            *p_state = STATE_INIT_WAIT;

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
    bool toggle;

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

        // Wait for the required amount of time.
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
        sensor_control[sensor].state           = STATE_POR;
        sensor_control[sensor].disabled        = false;
        sensor_control[sensor].init_wait_timer = 0;
        sensor_control[sensor].relay_position  = SWITCH_LBL;
        sensor_control[sensor].mode            = MODE_C_DIFFERENTIAL;
        sensor_control[sensor].conversion_time = CONVERT_TIME_109MS;
        sensor_control[sensor].enable_c1_c2    = false;

        /* Initialize the I2C bus for the sensor */
        I2CInit(sensor);
    }

    /* Testing: disable some sensors */
    sensor_control[SENSOR2].disabled = true;
    sensor_control[SENSOR3].disabled = true;
    sensor_control[SENSOR4].disabled = true;
    sensor_control[SENSOR5].disabled = true;
    //sensor_control[SENSOR6].disabled = true;

    if(xTaskCreate(Sensor_Task, (const portCHAR *)"SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                   tskIDLE_PRIORITY + PRIORITY_SENSOR_TASK, NULL) != pdTRUE) {
        return(1);
    }

    // Success.
    return(0);
}

