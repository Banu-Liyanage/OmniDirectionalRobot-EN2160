/*
 * pid.h
 *
 *  Created on: May 15, 2025
 *      Author: PANKAJA
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h" // Or specific types like float

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float prev_error;
    float integral;

    float output_min;
    float output_max;

    uint32_t last_time; // For calculating dt
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_min, float output_max);
float PID_Compute(PID_Controller *pid, float current_value, float dt); // dt in seconds
void PID_Reset(PID_Controller *pid);
void PID_SetSetpoint(PID_Controller *pid, float setpoint);
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd);

#endif /* INC_PID_H_ */
