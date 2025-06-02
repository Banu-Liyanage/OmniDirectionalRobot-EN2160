/*
 * motors.c
 *
 *  Created on: May 20, 2025
 *      Author: PANKAJA
 */

#include "motors.h"
#include "config.h"

float limitPWM(float pwm) {
    if (pwm > PWM_MAX)
        return PWM_MAX;
    else if (pwm < -PWM_MAX)
        return -PWM_MAX;
    return pwm;
}

void setForwardLeftMotorPWM(float pwm) {
    // Limit PWM value
    pwm = limitPWM(pwm);

    if (pwm >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, GPIO_PIN_SET);
        TIM1->CCR1 = (uint32_t)(pwm * MAX_TIMER_COUNTS);
    }
    else {
        // Reverse direction
        HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, GPIO_PIN_RESET);
        TIM1->CCR1 = (uint32_t)(-pwm * MAX_TIMER_COUNTS);
    }
}

void setForwardRightMotorPWM(float pwm) {
    // Limit PWM value
    pwm = limitPWM(pwm);

    if (pwm >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(M2_INA_GPIO_Port, M2_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M2_INB_GPIO_Port, M2_INB_Pin, GPIO_PIN_RESET);
        TIM1->CCR2 = (uint32_t)(pwm * MAX_TIMER_COUNTS);
    }
    else {
        // Reverse direction
        HAL_GPIO_WritePin(M2_INA_GPIO_Port, M2_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M2_INB_GPIO_Port, M2_INB_Pin, GPIO_PIN_SET);
        TIM1->CCR2 = (uint32_t)(-pwm * MAX_TIMER_COUNTS);
    }
}

void setRearLeftMotorPWM(float pwm) {
    // Limit PWM value
    pwm = limitPWM(pwm);

    if (pwm >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(M3_INA_GPIO_Port, M3_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M3_INB_GPIO_Port, M3_INB_Pin, GPIO_PIN_SET);
        TIM1->CCR3 = (uint32_t)(pwm * MAX_TIMER_COUNTS);
    }
    else {
        // Reverse direction
        HAL_GPIO_WritePin(M3_INA_GPIO_Port, M3_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M3_INB_GPIO_Port, M3_INB_Pin, GPIO_PIN_RESET);
        TIM1->CCR3 = (uint32_t)(-pwm * MAX_TIMER_COUNTS);
    }
}

void setRearRightMotorPWM(float pwm) {
    // Limit PWM value
    pwm = limitPWM(pwm);

    if (pwm >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(M4_INA_GPIO_Port, M4_INA_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M4_INB_GPIO_Port, M4_INB_Pin, GPIO_PIN_RESET);
        TIM1->CCR4 = (uint32_t)(pwm * MAX_TIMER_COUNTS);
    }
    else {
        // Reverse direction
        HAL_GPIO_WritePin(M4_INA_GPIO_Port, M4_INA_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M4_INB_GPIO_Port, M4_INB_Pin, GPIO_PIN_SET);
        TIM1->CCR4 = (uint32_t)(-pwm * MAX_TIMER_COUNTS);
    }
}

void resetMotors(void) {
    // Stop all motors
    setForwardLeftMotorPWM(0);
    setForwardRightMotorPWM(0);
    setRearLeftMotorPWM(0);
    setRearRightMotorPWM(0);

    // Set all control pins low
    HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(M2_INA_GPIO_Port, M2_INA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_INB_GPIO_Port, M2_INB_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(M3_INA_GPIO_Port, M3_INA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_INB_GPIO_Port, M3_INB_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(M4_INA_GPIO_Port, M4_INA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M4_INB_GPIO_Port, M4_INB_Pin, GPIO_PIN_RESET);
}

