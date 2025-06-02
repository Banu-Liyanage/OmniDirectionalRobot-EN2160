/*
 * kinematics.h
 *
 *  Created on: May 22, 2025
 *      Author: PANKAJA
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "stdint.h"
#include "config.h"

// Robot physical parameters (adjust these based on your robot)
#define WHEEL_RADIUS 0.05f        // Wheel radius in meters (50mm)
#define L1 0.12f                  // Distance from center to wheel axis (x-direction) in meters
#define L2 0.10f                  // Distance from center to wheel axis (y-direction) in meters

// Velocity limits (m/s and rad/s)
#define MAX_LINEAR_VELOCITY 1.0f   // Maximum linear velocity in m/s
#define MAX_ANGULAR_VELOCITY 3.14f // Maximum angular velocity in rad/s

// Control loop parameters
#define CONTROL_FREQUENCY 1000     // Control loop frequency in Hz (1kHz)
#define CONTROL_PERIOD 0.001f      // Control period in seconds (1ms)

// Demonstration movement parameters
#define DEMO_VELOCITY 0.3f         // Demo velocity in m/s
#define DEMO_ANGULAR_VEL 1.0f      // Demo angular velocity in rad/s
#define DEMO_DURATION 2000         // Demo duration in ms

// Robot body velocities structure
typedef struct {
    float vx;      // Linear velocity in x-direction (m/s)
    float vy;      // Linear velocity in y-direction (m/s)
    float wz;      // Angular velocity around z-axis (rad/s)
} robot_velocity_t;

// Wheel angular velocities structure
typedef struct {
    float w1;      // Front-left wheel angular velocity (rad/s)
    float w2;      // Front-right wheel angular velocity (rad/s)
    float w3;      // Rear-left wheel angular velocity (rad/s)
    float w4;      // Rear-right wheel angular velocity (rad/s)
} wheel_velocities_t;

// Demo movement states
typedef enum {
    DEMO_STOP = 0,
    DEMO_FORWARD,
    DEMO_BACKWARD,
    DEMO_LEFT,
    DEMO_RIGHT,
    DEMO_DIAGONAL_FL,
    DEMO_DIAGONAL_FR,
    DEMO_ROTATE_CW,
    DEMO_ROTATE_CCW,
    DEMO_STRAFE_ROTATE,
    DEMO_COMPLETE
} demo_state_t;

// Function prototypes
void kinematics_init(void);
void inverse_kinematics(robot_velocity_t body_vel, wheel_velocities_t* wheel_vel);
void forward_kinematics(wheel_velocities_t wheel_vel, robot_velocity_t* body_vel);
void set_robot_velocity(float vx, float vy, float wz);
void update_kinematics_control(void);
void start_demo_sequence(void);
void update_demo_sequence(void);
void stop_robot(void);
robot_velocity_t limit_velocities(robot_velocity_t vel);

// Global variables (extern declarations)
extern volatile robot_velocity_t current_robot_velocity;
extern volatile robot_velocity_t target_robot_velocity;
extern volatile wheel_velocities_t target_wheel_velocities;
extern volatile wheel_velocities_t current_wheel_velocities;
extern volatile demo_state_t demo_state;
extern volatile uint32_t demo_timer;

#endif /* KINEMATICS_H_ */
