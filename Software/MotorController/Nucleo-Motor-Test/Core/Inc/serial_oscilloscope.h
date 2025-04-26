/*
 * serial_oscilloscope.h
 *
 *  Created on: Apr 26, 2025
 *      Author: PANKAJA
 */

#ifndef INC_SERIAL_OSCILLOSCOPE_H_
#define INC_SERIAL_OSCILLOSCOPE_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Configuration for Serial Oscilloscope
 */
typedef struct {
    UART_HandleTypeDef *huart;      /* UART handle for communication */
    uint32_t sample_rate_ms;        /* Sampling interval in milliseconds */
    uint8_t max_channels;           /* Maximum number of channels to plot */
} SerialOscilloscope_Config_t;

/**
 * @brief Initialize the Serial Oscilloscope module
 *
 * @param config Pointer to configuration structure
 * @return HAL_StatusTypeDef HAL_OK if initialization successful
 */
HAL_StatusTypeDef SerialOscilloscope_Init(SerialOscilloscope_Config_t *config);

/**
 * @brief Send data values to Serial Oscilloscope
 *
 * @param data_array Array of values to send (one value per channel)
 * @param num_channels Number of channels to send
 * @return HAL_StatusTypeDef HAL_OK if transmission successful
 */
HAL_StatusTypeDef SerialOscilloscope_SendData(int16_t *data_array, uint8_t num_channels);

/**
 * @brief Send labeled data values to Serial Oscilloscope
 * Example: "x=100,y=200,z=300\r\n"
 *
 * @param labels Array of labels for each channel (e.g., "x", "y", "z")
 * @param data_array Array of values to send (one value per channel)
 * @param num_channels Number of channels to send
 * @return HAL_StatusTypeDef HAL_OK if transmission successful
 */
HAL_StatusTypeDef SerialOscilloscope_SendLabeledData(char **labels, int16_t *data_array, uint8_t num_channels);

/**
 * @brief Check if it's time to sample based on the configured sample rate
 *
 * @return uint8_t 1 if it's time to sample, 0 if not
 */
uint8_t SerialOscilloscope_IsTimeToSample(void);

#endif /* INC_SERIAL_OSCILLOSCOPE_H_ */
