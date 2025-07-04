/*
 * encoders.h
 *
 *  Created on: May 21, 2025
 *      Author: PANKAJA
 */

#ifndef INC_ENCODERS_H_
#define INC_ENCODERS_H_

#include "main.h"
#include "config.h"

extern volatile float m1_W;
extern volatile float m4_W;
extern volatile float m2_W;
extern volatile float m3_W;

extern volatile WheelVelocities current_wheel_W;

int16_t getForwardLeftEncoderCounts();
int16_t getForwardRightEncoderCounts();
int16_t getRearLeftEncoderCounts();
int16_t getRearRightEncoderCounts();

void resetEncoders();
void resetEncodersinSystick();
void update_Encoder_Data();


#endif /* INC_ENCODERS_H_ */
