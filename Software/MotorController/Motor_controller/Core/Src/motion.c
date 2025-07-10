/*
 * motion.c
 *
 *  Created on: Jul 10, 2025
 *      Author: PANKAJA
 */


#include "motion.h"
#include "config.h"


void Motion_Init(Motion *motion, Controller *controller, Profile *x_profile, Profile *y_profile, Profile *W_profile) {
    motion->controller = *controller;
    motion->x_profile = *x_profile;
    motion->y_profile = *y_profile;
    motion->W_profile = *W_profile;
}


void Motion_ResetDriveSystem(Motion *motion) {
	Motion_Stop(motion);
	Motion_DisableDrive(motion);
    resetEncoders();
    Profile_Reset(&(motion->x_profile));
    Profile_Reset(&(motion->y_profile));
    Profile_Reset(&(motion->W_profile));
    Controller_ResetControllers(&(motion->controller));
    Controller_EnableControllers(&(motion->controller));
}


void Motion_Update(Motion *motion) {
    Profile_Update(&(motion->x_profile));
    Profile_Update(&(motion->y_profile));
    Profile_Update(&(motion->W_profile));
}

void Motion_Stop(Motion *motion) {
	Controller_Stop();
}

void Motion_DisableDrive(Motion *motion) {
	Controller_DisableControllers(&(motion->controller));
}


float Motion_XPosition(Motion *motion) {
    return Profile_GetPosition(&(motion->x_profile));
}

float Motion_YPosition(Motion *motion) {
    return Profile_GetPosition(&(motion->y_profile));
}

float Motion_XVelocity(Motion *motion) {
    return Profile_GetSpeed(&(motion->x_profile));
}

float Motion_YVelocity(Motion *motion) {
    return Profile_GetSpeed(&(motion->y_profile));
}

float Motion_XAcceleration(Motion *motion) {
    return Profile_GetAcceleration(&(motion->x_profile));
}

float Motion_YAcceleration(Motion *motion) {
    return Profile_GetAcceleration(&(motion->y_profile));
}

void Motion_SetTargetXVelocity(Motion *motion, float velocity) {
    Profile_SetTargetSpeed(&(motion->x_profile), velocity);
}

void Motion_SetTargetYVelocity(Motion *motion, float velocity) {
    Profile_SetTargetSpeed(&(motion->y_profile), velocity);
}


float Motion_Angle(Motion *motion) {
    return Profile_GetPosition(&(motion->W_profile));
}

float Motion_Omega(Motion *motion) {
    return Profile_GetSpeed(&(motion->W_profile));
}

float Motion_Alpha(Motion *motion) {
    return Profile_GetAcceleration(&(motion->W_profile));
}



void Motion_Start_XMove(Motion *motion, float distance, float top_speed, float final_speed, float acceleration) {
    Profile_Start(&(motion->x_profile), distance, top_speed, final_speed, acceleration);
}

void Motion_Start_YMove(Motion *motion, float distance, float top_speed, float final_speed, float acceleration) {
    Profile_Start(&(motion->y_profile), distance, top_speed, final_speed, acceleration);
}

uint8_t Motion_XMoveFinished(Motion *motion) {
    return Profile_IsFinished(&(motion->x_profile));
}

uint8_t Motion_YMoveFinished(Motion *motion) {
    return Profile_IsFinished(&(motion->y_profile));
}


void Motion_X(Motion *motion, float distance) {
    Profile_Move(&(motion->x_profile), distance, 0.25, 0, 0.05);
    Profile_WaitUntilFinished(&(motion->x_profile));
}

void Motion_Y(Motion *motion, float distance) {
    Profile_Start(&(motion->y_profile), distance, vmax_Y, 0, amax_Y);
    Profile_WaitUntilFinished(&(motion->x_profile));
}

void Motion_Diagonal(Motion *motion, float distance) {
	Profile_Start(&(motion->x_profile), distance, vmax_X, 0, amax_X);
    Profile_Start(&(motion->y_profile), distance, vmax_Y, 0, amax_Y);
    Profile_WaitUntilFinished(&(motion->x_profile));
    Profile_WaitUntilFinished(&(motion->y_profile));
}
