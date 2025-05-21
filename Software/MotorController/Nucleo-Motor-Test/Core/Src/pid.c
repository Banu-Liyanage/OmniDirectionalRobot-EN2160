/*
 * pid.c
 *
 *  Created on: May 15, 2025
 *      Author: PANKAJA
 */


#include "pid.h"
#include <limits.h> // For FLT_MAX or similar if needed for integral windup, though clamping output is usually preferred

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float output_min, float output_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->last_time = 0; // Or HAL_GetTick() if starting immediately
}

float PID_Compute(PID_Controller *pid, float current_value, float dt) {
    if (dt <= 0.0f) {
        // Avoid division by zero or negative dt
        // Return previous output or 0, depending on desired behavior
        // For now, let's assume output doesn't change if dt is invalid
        // This might need refinement based on how dt is calculated and handled.
        // A simple approach for now:
        float last_output = pid->Kp * pid->prev_error + pid->integral + pid->Kd * 0; // Assuming derivative is 0 if no time elapsed
         if (last_output > pid->output_max) {
            last_output = pid->output_max;
        } else if (last_output < pid->output_min) {
            last_output = pid->output_min;
        }
        return last_output;
    }

    float error = pid->setpoint - current_value;

    // Proportional term
    float P_out = pid->Kp * error;

    // Integral term (with anti-windup)
    pid->integral += pid->Ki * error * dt;
    // Anti-windup: Clamp integral term if output is already saturated
    // A more robust way is to clamp the integral itself to prevent it from growing too large
    // This simple clamping is one way, but better anti-windup strategies exist.
    // For now, we clamp the final output. Let's adjust the integral directly.
    float potential_output_for_integral_check = P_out + pid->integral;
    if (potential_output_for_integral_check > pid->output_max) {
        pid->integral = pid->output_max - P_out; // Prevent integral from pushing output beyond max
    } else if (potential_output_for_integral_check < pid->output_min) {
        pid->integral = pid->output_min - P_out; // Prevent integral from pulling output below min
    }
    // Ensure integral itself also has some bounds if necessary, but clamping the output is key.

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float D_out = pid->Kd * derivative;

    // Calculate total output
    float output = P_out + pid->integral + D_out;

    // Clamp output to limits
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    pid->prev_error = error;

    return output;
}

void PID_Reset(PID_Controller *pid) {
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    // pid->setpoint = 0.0f; // Optional: reset setpoint or leave it
}

void PID_SetSetpoint(PID_Controller *pid, float setpoint) {
    pid->setpoint = setpoint;
}

void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
