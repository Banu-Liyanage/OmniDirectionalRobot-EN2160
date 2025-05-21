/*
 * controller.c
 *
 *  Created on: May 15, 2025
 *      Author: PANKAJA
 */

#include "controller.h"

// In main.c or a dedicated motor_control.c
#define ENCODER_PPR (1000.0f) // Pulses Per Revolution of your encoder
#define GEAR_RATIO (1.0f)     // If you have a gearbox
#define WHEEL_DIAMETER (0.065f) // meters, if you want linear speed
#define ENCODER_SAMPLING_TIME_S (0.01f) // e.g., 10ms, how often you calculate velocity

volatile int32_t current_encoder_counts = 0;
volatile int32_t prev_encoder_counts = 0;
volatile float motor_velocity_rps = 0.0f; // Revolutions Per Second
volatile float motor_velocity_rpm = 0.0f; // Revolutions Per Minute
// volatile float motor_linear_velocity_mps = 0.0f; // Meters Per Second (if applicable)

// Call this function periodically (e.g., in a timer interrupt or main loop with fixed timing)
void calculate_velocity(void) {
    current_encoder_counts = getEncoderCounts(); // This should read TIM1->CNT

    // Handle counter overflow/underflow if your encoder can move more than 32767 counts between samples
    // For a 16-bit timer (0-65535), if it's TIM_ENCODERMODE_TI1ANDTI2 (x4),
    // one full mechanical rotation might be PPR * 4 counts.
    // If your timer ARR is 65535, TIM1->CNT will be between 0 and 65535.
    // Your getEncoderCounts() casts to int16_t, which is -32768 to 32767.
    // It's better to work with uint16_t from the timer and handle wrap-around.

    // Let's assume getEncoderCounts() returns the direct 16-bit timer count (0-65535)
    // and we handle the difference carefully.
    // For simplicity, if using int16_t and resetting often, this might be okay,
    // but for continuous rotation, a more robust delta calculation is needed.

    // Using your existing int16_t getEncoderCounts():
    int16_t delta_counts = current_encoder_counts - prev_encoder_counts;

    // If you are NOT resetting the encoder count TIM1->CNT regularly,
    // you need to handle wrap-around for delta_counts with a 16-bit counter.
    // Example for a 16-bit up/down counter:
    // if (delta_counts > 32767) delta_counts -= 65536;
    // if (delta_counts < -32767) delta_counts += 65536;

    motor_velocity_rps = ((float)delta_counts / (ENCODER_PPR * 4.0f * GEAR_RATIO)) / ENCODER_SAMPLING_TIME_S; // *4 for x4 encoder mode
    motor_velocity_rpm = motor_velocity_rps * 60.0f;
    // motor_linear_velocity_mps = motor_velocity_rps * PI * WHEEL_DIAMETER;

    prev_encoder_counts = current_encoder_counts;
    // You might choose NOT to reset TIM1->CNT here if you are calculating delta_counts
    // Resetting it means you are always measuring absolute counts since last reset.
    // For continuous velocity, using the delta is more common.
    // If you reset: resetEncoders(); then prev_encoder_counts should also be 0.
}

// Update your getEncoderCounts to return the raw 16-bit value
uint16_t getRawEncoderCounts() {
    return (uint16_t)TIM1->CNT;
}

// Modified velocity calculation using raw counts and handling wrap-around
void calculate_velocity_robust(void) {
    static uint16_t last_raw_counts = 0;
    uint16_t current_raw_counts = getRawEncoderCounts();

    int32_t delta_counts_raw = (int32_t)current_raw_counts - (int32_t)last_raw_counts;

    // Handle wrap-around for a 16-bit counter (0-65535)
    if (delta_counts_raw > 32768) { // Arbitrary threshold, half of 65536
        delta_counts_raw -= 65536; // Underflowed
    } else if (delta_counts_raw < -32768) {
        delta_counts_raw += 65536; // Overflowed
    }

    motor_velocity_rps = ((float)delta_counts_raw / (ENCODER_PPR * 4.0f * GEAR_RATIO)) / ENCODER_SAMPLING_TIME_S;
    motor_velocity_rpm = motor_velocity_rps * 60.0f;

    last_raw_counts = current_raw_counts;
}
