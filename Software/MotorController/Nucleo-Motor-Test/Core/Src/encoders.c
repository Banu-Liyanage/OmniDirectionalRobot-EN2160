/*
 * encoders.c
 *
 *  Created on: Apr 25, 2025
 *      Author: PANKAJA
 */


#include "main.h"
#include "encoders.h"

int16_t getEncoderCounts() {
	return (int16_t) TIM1->CNT;
}

void resetEncoders() {
	TIM1->CNT = (int16_t) 0;
}
