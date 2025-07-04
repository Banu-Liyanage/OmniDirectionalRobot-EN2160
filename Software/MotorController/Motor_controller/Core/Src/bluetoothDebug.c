/*
 * bluetoothDebug.c
 *
 *  Created on: Jul 2, 2025
 *      Author: PANKAJA
 */

#include "bluetoothDebug.h"
#include "string.h"
#include "stdio.h"  // only for strlen
#include "stdlib.h"

// Convert integer to string
void int_to_str(int num, char *str)
{
    char temp[12];
    int i = 0, j = 0;
    int is_negative = 0;

    if (num < 0)
    {
        is_negative = 1;
        num = -num;
    }

    do {
        temp[i++] = (num % 10) + '0';
        num /= 10;
    } while (num != 0);

    if (is_negative)
        temp[i++] = '-';

    // Reverse string
    while (i > 0)
        str[j++] = temp[--i];

    str[j] = '\0';
}

// Convert float to string (basic, fixed decimal points)
void float_to_string(float num, char *str, uint8_t decimal_points)
{
    int int_part = (int)num;
    float fraction = num - int_part;
    if (fraction < 0) fraction = -fraction;

    char int_str[12];
    int_to_str(int_part, int_str);

    strcpy(str, int_str);
    int len = strlen(str);
    str[len++] = '.';

    for (uint8_t i = 0; i < decimal_points; i++)
    {
        fraction *= 10;
        int digit = (int)fraction;
        str[len++] = digit + '0';
        fraction -= digit;
    }

    str[len] = '\0';
}

// UART Initialization
void UART_Init(UART_HandleTypeDef *huart)
{
    HAL_UART_Init(huart);
}

// Transmit a string
void UART_Transmit(UART_HandleTypeDef *huart, char *data)
{
    HAL_UART_Transmit(huart, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
}

// Transmit float with header
void UART_Transmit_Float(UART_HandleTypeDef *huart, const char *header, float number, uint8_t decimal_points)
{
    char float_str[20];
    char buffer[60];

    float_to_string(number, float_str, decimal_points);

    strcpy(buffer, header);
    strcat(buffer, ":");
    strcat(buffer, float_str);
    strcat(buffer, "\r\n");

    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Transmit int with header
void UART_Transmit_Int(UART_HandleTypeDef *huart, const char *header, int number)
{
    char int_str[12];
    char buffer[60];

    int_to_str(number, int_str);

    strcpy(buffer, header);
    strcat(buffer, ":");
    strcat(buffer, int_str);
    strcat(buffer, "\r\n");

    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Transmit IR sensor values
void UART_Transmit_IR(UART_HandleTypeDef *huart, uint16_t IRL, uint16_t IRR)
{
    char buffer[70];
    char time_str[20], irl_str[12], irr_str[12];

    float timestamp = HAL_GetTick() / 1000.0f;

    float_to_string(timestamp, time_str, 2);
    int_to_str(IRL, irl_str);
    int_to_str(IRR, irr_str);

    strcpy(buffer, time_str);
    strcat(buffer, "\t");
    strcat(buffer, irl_str);
    strcat(buffer, "\t");
    strcat(buffer, irr_str);
    strcat(buffer, "\n");

    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Transmit TOF sensor values
void UART_Transmit_TOF(UART_HandleTypeDef *huart, uint16_t TOF1, uint16_t TOF2, uint16_t TOF3, uint16_t TOF4)
{
    char buffer[80];
    char t1[12], t2[12], t3[12], t4[12];

    int_to_str(TOF1, t1);
    int_to_str(TOF2, t2);
    int_to_str(TOF3, t3);
    int_to_str(TOF4, t4);

    strcpy(buffer, ">LW:");
    strcat(buffer, t1);
    strcat(buffer, ",LC:");
    strcat(buffer, t2);
    strcat(buffer, ",RC:");
    strcat(buffer, t3);
    strcat(buffer, ",RW:");
    strcat(buffer, t4);
    strcat(buffer, "\r\n");

    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void UART_Transmit_WheelW(UART_HandleTypeDef *huart, float W1, float W2, float W3, float W4)
{
    char buffer[80];
    char t1[12], t2[12], t3[12], t4[12];

    float_to_string(W1, t1, 3);
    float_to_string(W2, t2, 3);
    float_to_string(W3, t3, 3);
    float_to_string(W4, t4, 3);

    strcpy(buffer, ">FL:");
    strcat(buffer, t1);
    strcat(buffer, ",FR:");
    strcat(buffer, t2);
    strcat(buffer, ",RL:");
    strcat(buffer, t3);
    strcat(buffer, ",RR:");
    strcat(buffer, t4);
    strcat(buffer, "\r\n");

    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
