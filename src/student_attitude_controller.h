#ifndef STUDENT_ATTITUDE_CONTROLLER_H_
#define STUDENT_ATTITUDE_CONTROLLER_H_

#include "FreeRTOS.h"

#include "commander.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "student_pid.h"
#include <stdbool.h>
#include <stdint.h>

// PID integral limits.
// Set to 0 for no limit

// PID integral limits.
// These constants define the maximum accumulated error value (integral term)
// for each PID controller to prevent integral windup.
// Set to 0 for no limit, which disables integral anti-windup protection.

// Maximum integral term accumulation for roll angle controller
// Setting to 0.0f means no integral windup protection for roll angle
#define PID_ROLL_INTEGRAL_LIMIT (float)(0.0f)

// Maximum integral term accumulation for pitch angle controller
// Setting to 0.0f means no integral windup protection for pitch angle
#define PID_PITCH_INTEGRAL_LIMIT (float)(0.0f)

// Maximum integral term accumulation for yaw angle controller
// Setting to 0.0f means no integral windup protection for yaw angle
#define PID_YAW_INTEGRAL_LIMIT (float)(0.0f)

// Maximum integral term accumulation for roll rate controller
// Setting to 0.0f means no integral windup protection for roll rate
#define PID_ROLL_RATE_INTEGRAL_LIMIT (float)(0.0f)

// Maximum integral term accumulation for pitch rate controller
// Setting to 0.0f means no integral windup protection for pitch rate
#define PID_PITCH_RATE_INTEGRAL_LIMIT (float)(0.0f)

// Maximum integral term accumulation for yaw rate controller
// Setting to 0.0f means no integral windup protection for yaw rate
#define PID_YAW_RATE_INTEGRAL_LIMIT (float)(0.0f)

// Enable unit test mode (defines the log functions)
#define UNIT_TEST_MODE true

// Update rate of the inertial measurement unit
#define IMU_UPDATE_DT (float)(0.01f)
// Update rate of the attitude
#define ATTITUDE_RATE (float)(1.0f / IMU_UPDATE_DT)

// low pass filter settings
#define ATTITUDE_LPF_CUTOFF_FREQ 15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

/**
 * @brief Initialize PID data structures with coefficients defined in `pid.h`.
 *
 * @param updateDt expected delta time since last call for all PID loops
 */
void studentAttitudeControllerInit(const float updateDt);

/**
 * @brief Simple test to make sure controller is initialized
 *
 * @return true/false
 */
bool studentAttitudeControllerTest(void);

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 *
 * @param rollAngleMeasured input, the measured roll angle
 * @param pitchAngleMeasured input, the measured pitch angle
 * @param yawAngleMeasured input, the measured yaw angle
 * @param rollAngleDesired input, the desired roll angle
 * @param pitchAngleDesired input, the desired pitch angle
 * @param yawAngleDesired input, the desired yaw angle
 * @param rollRateDesired output, the calculated desired roll rate
 * @param pitchRateDesired output, the calculated desired pitch rate
 * @param yawRateDesired output, the calculated desired yaw rate
 */
void studentAttitudeControllerCorrectAttitudePID( //
    float rollAngleMeasured,                      //
    float pitchAngleMeasured,                     //
    float yawAngleMeasured,                       //
    float rollAngleDesired,                       //
    float pitchAngleDesired,                      //
    float yawAngleDesired,                        //
    float *rollRateDesired,                       //
    float *pitchRateDesired,                      //
    float *yawRateDesired                         //
);

/**
 * Make the controller run an update of the rate PID. Input comes from the
 * correct attitude function. The output is the actuator force.
 *
 * TODO MICROCART: Add output to this function to better line up with the other
 * controllers
 *
 * @param rollRateMeasured input, the measured roll rate
 * @param pitchRateMeasured input, the measured pitch rate
 * @param yawRateMeasured input, the measured yaw rate
 * @param rollRateDesired input, the desired roll rate
 * @param pitchRateDesired input, the desired pitch rate
 * @param yawRateDesired input, the desired yaw rate
 * @param rollCommand output, the calculated roll command
 * @param pitchCommand output, the calculated pitch command
 * @param yawCommand output, the calculated yaw command
 */
void studentAttitudeControllerCorrectRatePID( //
    float rollRateMeasured,                   //
    float pitchRateMeasured,                  //
    float yawRateMeasured,                    //
    float rollRateDesired,                    //
    float pitchRateDesired,                   //
    float yawRateDesired,                     //
    int16_t *rollCommand,                     //
    int16_t *pitchCommand,                    //
    int16_t *yawCommand                       //
);

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
