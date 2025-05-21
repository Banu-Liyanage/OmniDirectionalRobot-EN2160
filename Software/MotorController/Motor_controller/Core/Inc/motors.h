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

//#define M1_INA_PIN             10  // PC10
//#define M1_INB_PIN             11  // PC11
//
//#define M2_INA_PIN             8  // PC8
//#define M2_INB_PIN             9  // PC9
//
//#define M3_INA_PIN             4  // PB4
//#define M3_INB_PIN             5  // PB5
//
//#define M4_INA_PIN             2  // PC2
//#define M4_INB_PIN             3  // PC3


#define PWM_MAX 1 //  do not exceed 1
#define MAX_TIMER_COUNTS 7199



float limitPWM(float pwm);

void setForwardLeftMotorPWM(float pwm);
void setForwardRightMotorPWM(float pwm);
void setRearLeftMotorPWM(float pwm);
void setRearRightMotorPWM(float pwm);

void resetMotors();



#endif /* INC_MOTORS_H_ */
