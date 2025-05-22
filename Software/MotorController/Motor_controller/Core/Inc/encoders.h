/*
 * encoders.h
 *
 *  Created on: May 21, 2025
 *      Author: PANKAJA
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "main.h"

extern volatile float ForwardLeft_W;
extern volatile float ForwardRight_W;
extern volatile float RearLeft_W;
extern volatile float RearRight_W;

int16_t getForwardLeftEncoderCounts();
int16_t getForwardRightEncoderCounts();
int16_t getRearLeftEncoderCounts();
int16_t getRearRightEncoderCounts();

void resetEncoders();
void resetEncodersinSystick();
void update_Encoder_Data();




#endif /* INC_ENCODERS_H_ */
