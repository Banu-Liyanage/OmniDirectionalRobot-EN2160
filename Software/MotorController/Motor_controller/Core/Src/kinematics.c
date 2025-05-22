/*
 * kinematics.c
 *
 *  Created on: May 22, 2025
 *      Author: PANKAJA
 */

#include "kinematics.h"
#include "motors.h"
#include "encoders.h"
#include "math.h"

// Global variables
volatile robot_velocity_t current_robot_velocity = {0, 0, 0};
volatile robot_velocity_t target_robot_velocity = {0, 0, 0};
volatile wheel_velocities_t target_wheel_velocities = {0, 0, 0, 0};
volatile wheel_velocities_t current_wheel_velocities = {0, 0, 0, 0};
volatile demo_state_t demo_state = DEMO_STOP;
volatile uint32_t demo_timer = 0;

void kinematics_init(void) {
    // Initialize all velocities to zero
    current_robot_velocity.vx = 0;
    current_robot_velocity.vy = 0;
    current_robot_velocity.wz = 0;

    target_robot_velocity.vx = 0;
    target_robot_velocity.vy = 0;
    target_robot_velocity.wz = 0;

    target_wheel_velocities.w1 = 0;
    target_wheel_velocities.w2 = 0;
    target_wheel_velocities.w3 = 0;
    target_wheel_velocities.w4 = 0;

    current_wheel_velocities.w1 = 0;
    current_wheel_velocities.w2 = 0;
    current_wheel_velocities.w3 = 0;
    current_wheel_velocities.w4 = 0;

    demo_state = DEMO_STOP;
    demo_timer = 0;
}

// Inverse kinematics: Convert body velocities to wheel angular velocities
// Based on equation (9) from your reference
void inverse_kinematics(robot_velocity_t body_vel, wheel_velocities_t* wheel_vel) {
    float inv_R = 1.0f / WHEEL_RADIUS;
    float l1_plus_l2 = L1 + L2;

    // Apply kinematic transformation matrix (equation 9)
    // ω1 = (1/R) * [vx + vy - (l1+l2)*wz]  - Front Left
    wheel_vel->w1 = inv_R * (body_vel.vx + body_vel.vy - l1_plus_l2 * body_vel.wz);

    // ω2 = (1/R) * [vx - vy + (l1+l2)*wz]  - Front Right
    wheel_vel->w2 = inv_R * (body_vel.vx - body_vel.vy + l1_plus_l2 * body_vel.wz);

    // ω3 = (1/R) * [vx - vy - (l1+l2)*wz]  - Rear Left
    wheel_vel->w3 = inv_R * (body_vel.vx - body_vel.vy - l1_plus_l2 * body_vel.wz);

    // ω4 = (1/R) * [vx + vy + (l1+l2)*wz]  - Rear Right
    wheel_vel->w4 = inv_R * (body_vel.vx + body_vel.vy + l1_plus_l2 * body_vel.wz);
}

// Forward kinematics: Convert wheel angular velocities to body velocities
// Based on equation (8) from your reference
void forward_kinematics(wheel_velocities_t wheel_vel, robot_velocity_t* body_vel) {
    float R_over_4 = WHEEL_RADIUS / 4.0f;
    float l1_plus_l2 = L1 + L2;

    // Apply kinematic transformation matrix (equation 8)
    // vx = (R/4) * [ω1 + ω2 + ω3 + ω4]
    body_vel->vx = R_over_4 * (wheel_vel.w1 + wheel_vel.w2 + wheel_vel.w3 + wheel_vel.w4);

    // vy = (R/4) * [ω1 - ω2 - ω3 + ω4]
    body_vel->vy = R_over_4 * (wheel_vel.w1 - wheel_vel.w2 - wheel_vel.w3 + wheel_vel.w4);

    // wz = (R/4) * [-ω1/(l1+l2) + ω2/(l1+l2) - ω3/(l1+l2) + ω4/(l1+l2)]
    body_vel->wz = R_over_4 * (-wheel_vel.w1 + wheel_vel.w2 - wheel_vel.w3 + wheel_vel.w4) / l1_plus_l2;
}

// Limit velocities to safe ranges
robot_velocity_t limit_velocities(robot_velocity_t vel) {
    robot_velocity_t limited_vel = vel;

    // Limit linear velocities
    if (limited_vel.vx > MAX_LINEAR_VELOCITY) limited_vel.vx = MAX_LINEAR_VELOCITY;
    if (limited_vel.vx < -MAX_LINEAR_VELOCITY) limited_vel.vx = -MAX_LINEAR_VELOCITY;

    if (limited_vel.vy > MAX_LINEAR_VELOCITY) limited_vel.vy = MAX_LINEAR_VELOCITY;
    if (limited_vel.vy < -MAX_LINEAR_VELOCITY) limited_vel.vy = -MAX_LINEAR_VELOCITY;

    // Limit angular velocity
    if (limited_vel.wz > MAX_ANGULAR_VELOCITY) limited_vel.wz = MAX_ANGULAR_VELOCITY;
    if (limited_vel.wz < -MAX_ANGULAR_VELOCITY) limited_vel.wz = -MAX_ANGULAR_VELOCITY;

    return limited_vel;
}

// Set target robot velocity
void set_robot_velocity(float vx, float vy, float wz) {
    robot_velocity_t new_vel = {vx, vy, wz};
    new_vel = limit_velocities(new_vel);

    target_robot_velocity.vx = new_vel.vx;
    target_robot_velocity.vy = new_vel.vy;
    target_robot_velocity.wz = new_vel.wz;
}

// Stop the robot
void stop_robot(void) {
    set_robot_velocity(0, 0, 0);
}

// Main kinematics control update function (called from SysTick)
void update_kinematics_control(void) {
    // Update encoder data
    update_Encoder_Data();

    // Get current wheel velocities from encoders
    current_wheel_velocities.w1 = ForwardLeft_W;   // Front Left
    current_wheel_velocities.w2 = ForwardRight_W;  // Front Right
    current_wheel_velocities.w3 = RearLeft_W;      // Rear Left
    current_wheel_velocities.w4 = RearRight_W;     // Rear Right

    // Calculate current robot velocity using forward kinematics
    forward_kinematics(current_wheel_velocities, (robot_velocity_t*)&current_robot_velocity);

    // Convert target robot velocity to wheel velocities using inverse kinematics
    inverse_kinematics(target_robot_velocity, (wheel_velocities_t*)&target_wheel_velocities);

    // Convert wheel angular velocities to PWM values (simple proportional control)
    // You may want to implement PID control here for better performance
    float pwm_scale = 0.3f; // Adjust this scaling factor based on your system

    float pwm1 = target_wheel_velocities.w1 * pwm_scale;
    float pwm2 = target_wheel_velocities.w2 * pwm_scale;
    float pwm3 = target_wheel_velocities.w3 * pwm_scale;
    float pwm4 = target_wheel_velocities.w4 * pwm_scale;

    // Set motor PWM values
    setForwardLeftMotorPWM(pwm1);
    setForwardRightMotorPWM(pwm2);
    setRearLeftMotorPWM(pwm3);
    setRearRightMotorPWM(pwm4);
}

// Start demonstration sequence
void start_demo_sequence(void) {
    demo_state = DEMO_FORWARD;
    demo_timer = 0;
}

// Update demo sequence (called from SysTick)
void update_demo_sequence(void) {
    demo_timer++;

    // Check if it's time to switch to next demo state
    if (demo_timer >= DEMO_DURATION) {
        demo_timer = 0;
        demo_state++;

        if (demo_state >= DEMO_COMPLETE) {
            demo_state = DEMO_STOP;
        }
    }

    // Execute current demo state
    switch (demo_state) {
        case DEMO_STOP:
            set_robot_velocity(0, 0, 0);
            break;

        case DEMO_FORWARD:
            set_robot_velocity(DEMO_VELOCITY, 0, 0);  // Move forward
            break;

        case DEMO_BACKWARD:
            set_robot_velocity(-DEMO_VELOCITY, 0, 0); // Move backward
            break;

        case DEMO_LEFT:
            set_robot_velocity(0, DEMO_VELOCITY, 0);  // Strafe left
            break;

        case DEMO_RIGHT:
            set_robot_velocity(0, -DEMO_VELOCITY, 0); // Strafe right
            break;

        case DEMO_DIAGONAL_FL:
            set_robot_velocity(DEMO_VELOCITY, DEMO_VELOCITY, 0); // Diagonal forward-left
            break;

        case DEMO_DIAGONAL_FR:
            set_robot_velocity(DEMO_VELOCITY, -DEMO_VELOCITY, 0); // Diagonal forward-right
            break;

        case DEMO_ROTATE_CW:
            set_robot_velocity(0, 0, -DEMO_ANGULAR_VEL); // Rotate clockwise
            break;

        case DEMO_ROTATE_CCW:
            set_robot_velocity(0, 0, DEMO_ANGULAR_VEL);  // Rotate counter-clockwise
            break;

        case DEMO_STRAFE_ROTATE:
            set_robot_velocity(0, DEMO_VELOCITY, DEMO_ANGULAR_VEL); // Strafe left while rotating
            break;

        default:
            demo_state = DEMO_STOP;
            break;
    }
}
