/**
 * student_attitude_pid_controller.c: Attitude controller using PID correctors
 */
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#include "log.h"
#include "param.h"
#include "student_attitude_controller.h"
#include "student_pid.h"

// Update rate of the attitude
#define ATTITUDE_RATE (float)(1.0f / IMU_UPDATE_DT)

// low pass filter settings
#define ATTITUDE_LPF_CUTOFF_FREQ 15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

// structs to hold PID data between executions for each axis
PidObject pidRollRate;
PidObject pidPitchRate;
PidObject pidYawRate;
PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

/**
 * @brief Convert float to 16 bit integer
 * Use this for converting final value to store in the control struct
 *
 * @param in float
 * @return int16_t
 */
static inline int16_t saturateSignedInt16(float in) {
  // don't use INT16_MIN, because later we may negate it, which won't work for
  // that value.
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

// 488 TODO PidObject structs to hold PID data between executions for each axis

static bool isInit;

/**
 * @brief Initialize all PID data structures with PID coefficients defined in
 * student_pid.h
 *
 * @param updateDt expected delta time since last call for all PID loops
 */
void studentAttitudeControllerInit(const float updateDt) {
  if (isInit) {
    return;
  }

  // 488 TODO initialize all rate PID objects
  PidObject pidObj;

  studentPidInit(                    //
      &pidRollRate,                  //
      0,                             //
      PID_ROLL_RATE_KP,              //
      PID_ROLL_RATE_KI,              //
      PID_ROLL_RATE_KD,              //
      updateDt,                      //
      ATTITUDE_RATE,                 //
      ATTITUDE_RATE_LPF_CUTOFF_FREQ, //
      ATTITUDE_RATE_LPF_ENABLE       //
  );
  studentPidInit(                    //
      &pidPitchRate,                 //
      0,                             //
      PID_PITCH_RATE_KP,             //
      PID_PITCH_RATE_KI,             //
      PID_PITCH_RATE_KD,             //
      updateDt,                      //
      ATTITUDE_RATE,                 //
      ATTITUDE_RATE_LPF_CUTOFF_FREQ, //
      ATTITUDE_RATE_LPF_ENABLE       //
  );
  studentPidInit(                    //
      &pidYawRate,                   //
      0,                             //
      PID_YAW_RATE_KP,               //
      PID_YAW_RATE_KI,               //
      PID_YAW_RATE_KD,               //
      updateDt,                      //
      ATTITUDE_RATE,                 //
      ATTITUDE_RATE_LPF_CUTOFF_FREQ, //
      ATTITUDE_RATE_LPF_ENABLE       //
  );
  // studentPidInit(PidObject *pid, const float desired, const float kp, const
  // float ki, const float kd, const float dt, const float samplingRate, const
  // float cutoffFreq, bool enableDFilter)

  // 488 TODO set integral limits for all rate PID loops, 0 for no limit

  // 488 TODO initialize all attitude PID objects

  // 488 TODO set integral limits for attitude PID loops, 0 for no limit

  isInit = true;
}

/**
 * @brief Simple test to make sure controller is initialized
 *
 * @return true/false
 */
bool studentAttitudeControllerTest() { return isInit; }

/**
 * Make the controller run an update of the attitude PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * attitude controller can be run in a slower update rate then the rate
 * controller.
 *
 * @param eulerRollActual input
 * @param eulerPitchActual input
 * @param eulerYawActual input
 * @param eulerRollDesired input
 * @param eulerPitchDesired input
 * @param eulerYawDesired input
 * @param rollRateDesired output
 * @param pitchRateDesired output
 * @param yawRateDesired output
 */
void studentAttitudeControllerCorrectAttitudePID(
    float eulerRollActual, float eulerPitchActual, float eulerYawActual,
    float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
    float *rollRateDesired, float *pitchRateDesired, float *yawRateDesired) {

  // 488 TODO update all attitude PID's

  // 488 TODO Update PID for yaw axis, handle error update here instead of in
  // PID calculation to keep error between -180 and 180
}

/**
 * Make the controller run an update of the rate PID. Input comes from the
 * correct attitude function. The output is the actuator force.
 *  *
 * @param rollRateActual input
 * @param pitchRateActual input
 * @param yawRateActual input
 * @param rollRateDesired input
 * @param pitchRateDesired input
 * @param yawRateDesired input
 * @param rollCmd output
 * @param pitchCmd output
 * @param yawCmd
 */
void studentAttitudeControllerCorrectRatePID(
    float rollRateActual, float pitchRateActual, float yawRateActual,
    float rollRateDesired, float pitchRateDesired, float yawRateDesired,
    int16_t *rollCmd, int16_t *pitchCmd, int16_t *yawCmd) {

  // 488 TODO update all attitude rate PID's
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject *pid) {
  pid->prev_error = 0;
  pid->total_error = 0;
}

void studentAttitudeControllerResetRollAttitudePID(void) {
  studentPidReset(&pidRoll);
}

void studentAttitudeControllerResetYawAttitudePID(void) {
  studentPidReset(&pidYaw);
}

void studentAttitudeControllerResetPitchAttitudePID(void) {
  studentPidReset(&pidPitch);
}

void studentAttitudeControllerResetAllPID(void) {
  studentPidReset(&pidRoll);
  studentPidReset(&pidPitch);
  studentPidReset(&pidYaw);
  studentPidReset(&pidRollRate);
  studentPidReset(&pidPitchRate);
  studentPidReset(&pidYawRate);
}

// 488 TODO setup logging parameters, replace null with pointer to globabl
// variable

/**
 *  Log variables of attitude PID controller
 */
LOG_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, NULL)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, NULL)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, NULL)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, NULL)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, NULL)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, NULL)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, NULL)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, NULL)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, NULL)
LOG_GROUP_STOP(s_pid_attitude)

// 488 TODO setup logging parameters, replace null with pointer to globabl
// variable

/**
 *  Log variables of attitude rate PID controller
 */
LOG_GROUP_START(s_pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, NULL)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, NULL)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, NULL)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, NULL)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, NULL)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, NULL)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, NULL)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, NULL)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, NULL)
LOG_GROUP_STOP(s_pid_rate)

// 488 TODO setup adjustment parameters, replace null with pointer to globabl
// variables

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll
 */
PARAM_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, NULL)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, NULL)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, NULL)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, NULL)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, NULL)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, NULL)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, NULL)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, NULL)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, NULL)
PARAM_GROUP_STOP(s_pid_attitude)

// 488 TODO setup adjustment parameters, replace null with pointer to globabl
// variables

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates
 */
PARAM_GROUP_START(s_pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, NULL)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, NULL)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, NULL)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, NULL)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, NULL)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, NULL)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, NULL)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, NULL)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, NULL)
PARAM_GROUP_STOP(s_pid_rate)
