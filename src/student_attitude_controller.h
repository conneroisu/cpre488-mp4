#ifndef STUDENT_ATTITUDE_CONTROLLER_H_
#define STUDENT_ATTITUDE_CONTROLLER_H_

#include <stdbool.h>
#include "commander.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include "student_pid.h"

// Maximum integral term accumulation for roll rate controller
// Setting to 0.0f means no integral windup protection for roll rate
#define PID_ROLL_RATE_INTEGRAL_LIMIT (float)(33.3f)

// Maximum integral term accumulation for pitch rate controller
// Setting to 0.0f means no integral windup protection for pitch rate
#define PID_PITCH_RATE_INTEGRAL_LIMIT (float)(33.0f)

// Maximum integral term accumulation for yaw rate controller
// Setting to 0.0f means no integral windup protection for yaw rate
#define PID_YAW_RATE_INTEGRAL_LIMIT (float)(166.7f)

// Maximum integral term accumulation for roll angle controller
// Setting to 0.0f means no integral windup protection for roll angle
#define PID_ROLL_INTEGRAL_LIMIT (float)(20.0f)

// Maximum integral term accumulation for pitch angle controller
// Setting to 0.0f means no integral windup protection for pitch angle
#define PID_PITCH_INTEGRAL_LIMIT (float)(20.0f)

// Maximum integral term accumulation for yaw angle controller
// Setting to 0.0f means no integral windup protection for yaw angle
#define PID_YAW_INTEGRAL_LIMIT (float)(360.0f)

// low pass filter settings
#define ATTITUDE_LPF_CUTOFF_FREQ 15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

void studentAttitudeControllerInit(const float updateDt);

bool studentAttitudeControllerTest(void);

void studentAttitudeControllerCorrectAttitudePID(
    float eulerRollActual, float eulerPitchActual, float eulerYawActual,
    float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
    float *rollRateDesired, float *pitchRateDesired, float *yawRateDesired);

void studentAttitudeControllerCorrectRatePID(
    float rollRateActual, float pitchRateActual, float yawRateActual,
    float rollRateDesired, float pitchRateDesired, float yawRateDesired,
    int16_t *rollCmd, int16_t *pitchCmd, int16_t *yawCmd);

/**
 * Reset controller roll attitude PID
 */
void studentAttitudeControllerResetRollAttitudePID(void);

/**
 * Reset controller pitch attitude PID
 */
void studentAttitudeControllerResetPitchAttitudePID(void);

/**
 * Reset controller yaw attitude PID  
 */
void studentAttitudeControllerResetYawAttitudePID(void);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void studentAttitudeControllerResetAllPID(void);

#endif /* STUDENT_ATTITUDE_CONTROLLER_H_ */
