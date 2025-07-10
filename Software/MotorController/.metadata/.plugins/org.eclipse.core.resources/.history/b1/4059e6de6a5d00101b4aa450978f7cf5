/*
 * profile.c
 *
 *  Created on: Jul 4, 2025
 *      Author: PANKAJA
 */

#include "profile.h"
#include "bluetoothDebug.h"

extern UART_HandleTypeDef huart2;

// Reset the profile
void Profile_Reset(Profile *profile) {
    profile->position = 0;
    profile->speed = 0;
    profile->target_speed = 0;
    profile->state = PS_IDLE;
}

// Check if the profile has finished
uint8_t Profile_IsFinished(const Profile *profile) {
    return profile->state == PS_FINISHED;
}

// Start a profile
void Profile_Start(Profile *profile, float distance, float top_speed, float final_speed, float acceleration) {
    profile->sign = (distance < 0) ? -1 : 1;
    if (distance < 0) distance = -distance;

    if (distance < 1.0f) {
        profile->state = PS_FINISHED;
        return;
    }

    if (final_speed > top_speed) {
        final_speed = top_speed;
    }

    profile->position = 0;
    profile->final_position = distance;
    profile->target_speed = profile->sign * fabsf(top_speed);
    profile->final_speed = profile->sign * fabsf(final_speed);
    profile->acceleration = fabsf(acceleration);
    profile->one_over_acc = (profile->acceleration >= 1) ? (1.0f / profile->acceleration) : 1.0f;
    profile->state = PS_ACCELERATING;
}

// Get the braking distance
float Profile_GetBrakingDistance(const Profile *profile) {
    return fabsf(profile->speed * profile->speed - profile->final_speed * profile->final_speed) * 0.5f * profile->one_over_acc;
}

// Get the current position
float Profile_GetPosition(const Profile *profile) {
    return profile->position;
}

// Update the profile
void Profile_Update(Profile *profile) {
    if (profile->state == PS_IDLE) return;

    float delta_v = profile->acceleration * CONTROLLER_LOOP_INTERVAL;
    float remaining = fabsf(profile->final_position) - fabsf(profile->position);

    if (profile->state == PS_ACCELERATING) {
        if (remaining < Profile_GetBrakingDistance(profile)) {
            profile->state = PS_BRAKING;
            profile->target_speed = (profile->final_speed == 0) ? (profile->sign * 5.0f) : profile->final_speed;
        }
    }

    if (profile->speed < profile->target_speed) {
        profile->speed += delta_v;
        if (profile->speed > profile->target_speed) profile->speed = profile->target_speed;
    } else if (profile->speed > profile->target_speed) {
        profile->speed -= delta_v;
        if (profile->speed < profile->target_speed) profile->speed = profile->target_speed;
    }

    profile->position += profile->speed * CONTROLLER_LOOP_INTERVAL;

    if (profile->state != PS_FINISHED && remaining < 0.125f) {
        profile->state = PS_FINISHED;
        profile->target_speed = profile->final_speed;
    }
    //UART_Transmit_Float(&huart2, ">V", profile->speed, 2);

}

// Move a profile (blocking call)
void Profile_Move(Profile *profile, float distance, float top_speed, float final_speed, float acceleration) {
    Profile_Start(profile, distance, top_speed, final_speed, acceleration);
    //Profile_WaitUntilFinished(profile);
}
// Wait until the profile finishes
void Profile_WaitUntilFinished(Profile *profile) {
    while (profile->state != PS_FINISHED) {
        HAL_Delay(2);
    }
}

// Get the current speed
float Profile_GetSpeed(const Profile *profile) {
    return profile->speed;
}


