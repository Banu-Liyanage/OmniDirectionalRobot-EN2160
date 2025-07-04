/*
 * config.h
 *
 *  Created on: May 2, 2025
 *      Author: PANKAJA
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>

// Mathematical constants
#define PI 3.14159265358979323846f

// Mecanum wheel geometry constants
#define WHEEL_RADIUS 39.0f          // Wheel radius in mm (Diameter 78mm)
#define WHEEL_BASE_LENGTH 280.0f    // Distance between front and rear wheels (mm)
#define WHEEL_BASE_WIDTH 380.0f     // Distance between left and right wheels (mm)

// Robot physical constants
extern const float ROBOT_RADIUS;
extern const float DEG_PER_MM_DIFFERENCE;
extern const float RADIANS_PER_DEGREE;
extern const float DEGREES_PER_RADIAN;

// Motion controller constants
extern const float LOOP_FREQUENCY;
extern const float LOOP_INTERVAL;

extern const float Kp_Vel;
extern const float Ki_Vel;

extern const float MAXINTCLAMP;

// Motor control constants
#define MAX_TIMER_COUNTS 7199

// Encoder constants
#define ENCODER_COUNTS_PER_REVOLUTION 6800
#define WHEEL_CIRCUMFERENCE (2.0f * PI * 39.0f) // 39mm wheel radius


#define RAD_PER_COUNT_FORWARD_LEFT   (2.0f * PI / (ENCODER_COUNTS_PER_REVOLUTION))
#define RAD_PER_COUNT_FORWARD_RIGHT  (2.0f * PI / (ENCODER_COUNTS_PER_REVOLUTION))
#define RAD_PER_COUNT_REAR_LEFT      (2.0f * PI / (ENCODER_COUNTS_PER_REVOLUTION))
#define RAD_PER_COUNT_REAR_RIGHT     (2.0f * PI / (ENCODER_COUNTS_PER_REVOLUTION))


#define MM_PER_COUNT_FORWARD_LEFT    (WHEEL_CIRCUMFERENCE / (ENCODER_COUNTS_PER_REVOLUTION))
#define MM_PER_COUNT_FORWARD_RIGHT   (WHEEL_CIRCUMFERENCE / (ENCODER_COUNTS_PER_REVOLUTION))
#define MM_PER_COUNT_REAR_LEFT       (WHEEL_CIRCUMFERENCE / (ENCODER_COUNTS_PER_REVOLUTION))
#define MM_PER_COUNT_REAR_RIGHT      (WHEEL_CIRCUMFERENCE / (ENCODER_COUNTS_PER_REVOLUTION))


extern volatile float m1_W;
extern volatile float m4_W;
extern volatile float m2_W;
extern volatile float m3_W;



float clampf(float val, float min, float max);


#endif /* CONFIG_H_ */
