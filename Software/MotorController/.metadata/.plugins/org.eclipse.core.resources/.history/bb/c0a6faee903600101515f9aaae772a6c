/*
 * motors.c
 *
 *  Created on: May 20, 2025
 *      Author: PANKAJA
 */

#include "motors.h"

float limitPWM(float pwm) {
	if (pwm > PWM_MAX)
		return PWM_MAX;
	else if (pwm < -PWM_MAX)
		return -PWM_MAX;
	return pwm;
}


void setForwardLeftMotorPWM(float pwm) {
	if (pwm >= 0) {
		//GPIOB_BSRR = (1U << INA_PIN) | (1U << (INB_PIN + 16));
		HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, 1);
		HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, 0);

		TIM1->CCR1 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		//GPIOB_BSRR = (1U << (INA_PIN + 16)) | (1U << INB_PIN);
		HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, 0);
		HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, 1);

		TIM1->CCR1 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void setForwardRightMotorPWM(float pwm) {
	if (pwm >= 0) {
		//GPIOB_BSRR = (1U << INA_PIN) | (1U << (INB_PIN + 16));
		HAL_GPIO_WritePin(M2_INA_GPIO_Port, M2_INA_Pin, 1);
		HAL_GPIO_WritePin(M2_INB_GPIO_Port, M2_INB_Pin, 0);

		TIM1->CCR2 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		//GPIOB_BSRR = (1U << (INA_PIN + 16)) | (1U << INB_PIN);
		HAL_GPIO_WritePin(M2_INA_GPIO_Port, M2_INA_Pin, 0);
		HAL_GPIO_WritePin(M2_INB_GPIO_Port, M2_INB_Pin, 1);

		TIM1->CCR2 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void setRearLeftMotorPWM(float pwm) {
	if (pwm >= 0) {
		//GPIOB_BSRR = (1U << INA_PIN) | (1U << (INB_PIN + 16));
		HAL_GPIO_WritePin(M3_INA_GPIO_Port, M3_INA_Pin, 1);
		HAL_GPIO_WritePin(M3_INB_GPIO_Port, M3_INB_Pin, 0);

		TIM1->CCR3 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		//GPIOB_BSRR = (1U << (INA_PIN + 16)) | (1U << INB_PIN);
		HAL_GPIO_WritePin(M3_INA_GPIO_Port, M3_INA_Pin, 0);
		HAL_GPIO_WritePin(M3_INB_GPIO_Port, M3_INB_Pin, 1);

		TIM1->CCR3 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}

void setRearRightMotorPWM(float pwm) {
	if (pwm >= 0) {
		//GPIOB_BSRR = (1U << INA_PIN) | (1U << (INB_PIN + 16));
		HAL_GPIO_WritePin(M4_INA_GPIO_Port, M4_INA_Pin, 1);
		HAL_GPIO_WritePin(M4_INB_GPIO_Port, M4_INB_Pin, 0);

		TIM1->CCR4 = (uint32_t) (limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
	else if (pwm < 0) {
		//GPIOB_BSRR = (1U << (INA_PIN + 16)) | (1U << INB_PIN);
		HAL_GPIO_WritePin(M4_INA_GPIO_Port, M4_INA_Pin, 0);
		HAL_GPIO_WritePin(M4_INB_GPIO_Port, M4_INB_Pin, 1);

		TIM1->CCR4 = (uint32_t) (-1 * limitPWM(pwm) * MAX_TIMER_COUNTS);
	}
}




void resetMotors() {
	//GPIOB_BSRR = (1U << (INA_PIN + 16)) | (1U << (INB_PIN + 16));
	HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, 0);
	HAL_GPIO_WritePin(M1_INB_GPIO_Port, M1_INB_Pin, 0);

	setForwardLeftMotorPWM(0);
}
