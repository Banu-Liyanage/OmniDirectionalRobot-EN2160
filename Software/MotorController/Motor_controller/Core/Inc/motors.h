/*
 * motors.h
 *
 *  Created on: May 20, 2025
 *      Author: PANKAJA
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx.h"
#include "main.h"

#define NUM_SAMPLES 128

#define PWM_MAX 0.99
#define MAX_TIMER_COUNTS 7199


extern volatile float m1_target_W;
extern volatile float m2_target_W;
extern volatile float m3_target_W;
extern volatile float m4_target_W;

extern volatile float m1_int;
extern volatile float m2_int;
extern volatile float m3_int;
extern volatile float m4_int;



float limitPWM(float pwm);

void setForwardLeftMotorPWM(float pwm);
void setForwardRightMotorPWM(float pwm);
void setRearLeftMotorPWM(float pwm);
void setRearRightMotorPWM(float pwm);

void resetMotors();
void emergencyStop(void);
void initMotors(void);

uint16_t analogRead();
uint16_t getCurrentMilliamps();

void updateMotors();

void resetIntegralTerms(void);
void setTargetVelocities(float m1_target, float m2_target, float m3_target, float m4_target);

#endif /* INC_MOTORS_H_ */
