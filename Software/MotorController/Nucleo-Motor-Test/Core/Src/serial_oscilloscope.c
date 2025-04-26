/*
 * serial_oscilloscope.c
 *
 *  Created on: Apr 26, 2025
 *      Author: PANKAJA
 */


/**
 * @file serial_oscilloscope.c
 * @brief Implementation of Serial Oscilloscope interface for STM32 devices
 */

#include "serial_oscilloscope.h"
#include <stdio.h>
#include <string.h>

/* Private variables */
static SerialOscilloscope_Config_t osc_config;
static uint32_t last_sample_time = 0;
static char tx_buffer[128];  /* Buffer for UART transmission */

/**
 * @brief Initialize the Serial Oscilloscope module
 */
HAL_StatusTypeDef SerialOscilloscope_Init(SerialOscilloscope_Config_t *config)
{
    /* Check for null pointer */
    if (config == NULL || config->huart == NULL) {
        return HAL_ERROR;
    }

    /* Copy configuration parameters */
    memcpy(&osc_config, config, sizeof(SerialOscilloscope_Config_t));

    /* Initialize timing */
    last_sample_time = HAL_GetTick();

    return HAL_OK;
}

/**
 * @brief Send data values to Serial Oscilloscope
 */
HAL_StatusTypeDef SerialOscilloscope_SendData(int16_t *data_array, uint8_t num_channels)
{
    uint8_t i;
    int len = 0;

    /* Validate parameters */
    if (data_array == NULL || num_channels == 0 ||
        num_channels > osc_config.max_channels) {
        return HAL_ERROR;
    }

    /* Format data as comma-separated values with carriage return */
    for (i = 0; i < num_channels; i++) {
        if (i == 0) {
            len += sprintf(tx_buffer + len, "%d", data_array[i]);
        } else {
            len += sprintf(tx_buffer + len, ",%d", data_array[i]);
        }
    }

    /* Add line ending */
    len += sprintf(tx_buffer + len, "\r\n");

    /* Send over UART */
    return HAL_UART_Transmit(osc_config.huart, (uint8_t*)tx_buffer, len, 100);
}

/**
 * @brief Send labeled data values to Serial Oscilloscope
 */
HAL_StatusTypeDef SerialOscilloscope_SendLabeledData(char **labels, int16_t *data_array, uint8_t num_channels)
{
    uint8_t i;
    int len = 0;

    /* Validate parameters */
    if (data_array == NULL || labels == NULL || num_channels == 0 ||
        num_channels > osc_config.max_channels) {
        return HAL_ERROR;
    }

    /* Format data as labeled, comma-separated values */
    for (i = 0; i < num_channels; i++) {
        if (i == 0) {
            len += sprintf(tx_buffer + len, ">%s:%d", labels[i], data_array[i]);
        } else {
            len += sprintf(tx_buffer + len, ",%s:%d", labels[i], data_array[i]);
        }
    }

    /* Add line ending */
    len += sprintf(tx_buffer + len, "\r\n");

    /* Send over UART */
    return HAL_UART_Transmit(osc_config.huart, (uint8_t*)tx_buffer, len, 100);
}

/**
 * @brief Check if it's time to sample based on the configured sample rate
 */
uint8_t SerialOscilloscope_IsTimeToSample(void)
{
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_sample_time >= osc_config.sample_rate_ms) {
        last_sample_time = current_time;
        return 1;
    }

    return 0;
}
