#include <stdbool.h>
#include "FreeRTOS.h"
#include "student_attitude_controller.h"
#include "student_pid.h"
#include "param.h"
#include "log.h"
#include <math.h>

#define ATTITUDE_LPF_CUTOFF_FREQ 15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false

// PID objects
static PidObject pidRollRate;
static PidObject pidPitchRate;
static PidObject pidYawRate;

static PidObject pidRoll;
static PidObject pidPitch;
static PidObject pidYaw;

static bool isInit;

// Logging variables
static float roll_outP, roll_outI, roll_outD;
static float pitch_outP, pitch_outI, pitch_outD;
static float yaw_outP, yaw_outI, yaw_outD;
static float rollRate_outP, rollRate_outI, rollRate_outD;
static float pitchRate_outP, pitchRate_outI, pitchRate_outD;
static float yawRate_outP, yawRate_outI, yawRate_outD;

static inline int16_t saturateSignedInt16(float in) {
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

void studentAttitudeControllerInit(const float updateDt) {
  if (isInit)
    return;

  // Initialize rate PIDs (inner loop)
  studentPidInit(&pidRollRate, 0, PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidRollRate, 20.0f);
  
  studentPidInit(&pidPitchRate, 0, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidPitchRate, 20.0f);

  studentPidInit(&pidYawRate, 0, PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_RATE_LPF_CUTOFF_FREQ, ATTITUDE_RATE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidYawRate, 20.0f);

  // Initialize attitude PIDs (outer loop)
  studentPidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidRoll, 10.0f);

  studentPidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidPitch, 10.0f);

  studentPidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
                updateDt, 1.0f/updateDt, ATTITUDE_LPF_CUTOFF_FREQ, ATTITUDE_LPF_ENABLE);
  studentPidSetIntegralLimit(&pidYaw, 10.0f);

  isInit = true;
}

bool studentAttitudeControllerTest() {
  return isInit;
}

static float capAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void studentAttitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  // Calculate attitude errors
  float rollError = eulerRollDesired - eulerRollActual;
  float pitchError = eulerPitchDesired - eulerPitchActual;
  float yawError = capAngle(eulerYawDesired - eulerYawActual);

  // Update attitude PIDs and get desired rates
  *rollRateDesired = studentPidUpdate(&pidRoll, rollError, true);
  *pitchRateDesired = studentPidUpdate(&pidPitch, pitchError, true);
  *yawRateDesired = studentPidUpdate(&pidYaw, yawError, true);

  // Store PID outputs for logging
  roll_outP = pidRoll.kp * rollError;
  roll_outI = pidRoll.ki * pidRoll.total_error;
  roll_outD = pidRoll.kd * (rollError - pidRoll.prev_error) / pidRoll.dt;

  pitch_outP = pidPitch.kp * pitchError;
  pitch_outI = pidPitch.ki * pidPitch.total_error;
  pitch_outD = pidPitch.kd * (pitchError - pidPitch.prev_error) / pidPitch.dt;

  yaw_outP = pidYaw.kp * yawError;
  yaw_outI = pidYaw.ki * pidYaw.total_error;
  yaw_outD = pidYaw.kd * (yawError - pidYaw.prev_error) / pidYaw.dt;
}

void studentAttitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired,
       int16_t* rollCmd, int16_t* pitchCmd, int16_t* yawCmd)
{
  // Calculate rate errors
  float rollRateError = rollRateDesired - rollRateActual;
  float pitchRateError = pitchRateDesired - pitchRateActual;
  float yawRateError = yawRateDesired - yawRateActual;

  // Update rate PIDs and get motor commands
  float rollCmdRaw = studentPidUpdate(&pidRollRate, rollRateError, true);
  float pitchCmdRaw = studentPidUpdate(&pidPitchRate, pitchRateError, true);
  float yawCmdRaw = studentPidUpdate(&pidYawRate, yawRateError, true);

  // Saturate outputs to int16 range
  *rollCmd = saturateSignedInt16(rollCmdRaw);
  *pitchCmd = saturateSignedInt16(pitchCmdRaw);
  *yawCmd = saturateSignedInt16(yawCmdRaw);

  // Store PID outputs for logging
  rollRate_outP = pidRollRate.kp * rollRateError;
  rollRate_outI = pidRollRate.ki * pidRollRate.total_error;
  rollRate_outD = pidRollRate.kd * (rollRateError - pidRollRate.prev_error) / pidRollRate.dt;

  pitchRate_outP = pidPitchRate.kp * pitchRateError;
  pitchRate_outI = pidPitchRate.ki * pidPitchRate.total_error;
  pitchRate_outD = pidPitchRate.kd * (pitchRateError - pidPitchRate.prev_error) / pidPitchRate.dt;

  yawRate_outP = pidYawRate.kp * yawRateError;
  yawRate_outI = pidYawRate.ki * pidYawRate.total_error;
  yawRate_outD = pidYawRate.kd * (yawRateError - pidYawRate.prev_error) / pidYawRate.dt;
}

// Reset functions
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

// Logging setup
LOG_GROUP_START(s_pid_attitude)
LOG_ADD(LOG_FLOAT, roll_outP, &roll_outP)
LOG_ADD(LOG_FLOAT, roll_outI, &roll_outI)
LOG_ADD(LOG_FLOAT, roll_outD, &roll_outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pitch_outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pitch_outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pitch_outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &yaw_outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &yaw_outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &yaw_outD)
LOG_GROUP_STOP(s_pid_attitude)

LOG_GROUP_START(s_pid_rate)
LOG_ADD(LOG_FLOAT, roll_outP, &rollRate_outP)
LOG_ADD(LOG_FLOAT, roll_outI, &rollRate_outI)
LOG_ADD(LOG_FLOAT, roll_outD, &rollRate_outD)
LOG_ADD(LOG_FLOAT, pitch_outP, &pitchRate_outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pitchRate_outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pitchRate_outD)
LOG_ADD(LOG_FLOAT, yaw_outP, &yawRate_outP)
LOG_ADD(LOG_FLOAT, yaw_outI, &yawRate_outI)
LOG_ADD(LOG_FLOAT, yaw_outD, &yawRate_outD)
LOG_GROUP_STOP(s_pid_rate)

// Parameter setup
PARAM_GROUP_START(s_pid_attitude)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRoll.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRoll.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRoll.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYaw.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYaw.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYaw.kd)
PARAM_GROUP_STOP(s_pid_attitude)

PARAM_GROUP_START(s_pid_rate)
PARAM_ADD(PARAM_FLOAT, roll_kp, &pidRollRate.kp)
PARAM_ADD(PARAM_FLOAT, roll_ki, &pidRollRate.ki)
PARAM_ADD(PARAM_FLOAT, roll_kd, &pidRollRate.kd)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_ADD(PARAM_FLOAT, yaw_kp, &pidYawRate.kp)
PARAM_ADD(PARAM_FLOAT, yaw_ki, &pidYawRate.ki)
PARAM_ADD(PARAM_FLOAT, yaw_kd, &pidYawRate.kd)
PARAM_GROUP_STOP(s_pid_rate)
