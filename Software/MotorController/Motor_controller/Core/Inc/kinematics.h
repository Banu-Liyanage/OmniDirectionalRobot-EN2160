/*
 * kinematics.h
 *
 *  Created on: May 15, 2025
 *      Author: PANKAJA
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include <math.h>

// Robot velocity structure
typedef struct {
    float vx;    // Linear velocity in x direction (m/s)
    float vy;    // Linear velocity in y direction (m/s)
    float wz;    // Angular velocity around z axis (rad/s)
} RobotVelocity;




#endif /* KINEMATICS_H_ */
