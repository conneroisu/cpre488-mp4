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

static PidObject pidRollRate;
static PidObject pidPitchRate;
static PidObject pidYawRate;

static PidObject pidRoll;
static PidObject pidPitch;
static PidObject pidYaw;

static bool isInit;

static inline int16_t saturateSignedInt16(float in)
{
  if (in > INT16_MAX)
    return INT16_MAX;
  else if (in < -INT16_MAX)
    return -INT16_MAX;
  else
    return (int16_t)in;
}

void studentAttitudeControllerInit(const float updateDt)
{
  if (isInit)
    return;

  studentPidInit(&pidRollRate, RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD, updateDt);
  studentPidSetIntegralLimit(&pidRollRate, 0);
  
  studentPidInit(&pidPitchRate, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD, updateDt);
  studentPidSetIntegralLimit(&pidPitchRate, 0);

  studentPidInit(&pidYawRate, RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD, updateDt);
  studentPidSetIntegralLimit(&pidYawRate, 0);

  studentPidInit(&pidRoll, ATT_ROLL_KP, ATT_ROLL_KI, ATT_ROLL_KD, updateDt);
  studentPidSetIntegralLimit(&pidRoll, 0);

  studentPidInit(&pidPitch, ATT_PITCH_KP, ATT_PITCH_KI, ATT_PITCH_KD, updateDt);
  studentPidSetIntegralLimit(&pidPitch, 0);

  studentPidInit(&pidYaw, ATT_YAW_KP, ATT_YAW_KI, ATT_YAW_KD, updateDt);
  studentPidSetIntegralLimit(&pidYaw, 0);

  isInit = true;
}

bool studentAttitudeControllerTest()
{
  return isInit;
}

static float capAngle(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void studentAttitudeControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired)
{
  *rollRateDesired = studentPidUpdate(&pidRoll, eulerRollDesired - eulerRollActual);
  *pitchRateDesired = studentPidUpdate(&pidPitch, eulerPitchDesired - eulerPitchActual);

  float yawError = capAngle(eulerYawDesired - eulerYawActual);
  *yawRateDesired = studentPidUpdate(&pidYaw, yawError);
}

void studentAttitudeControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired,
       int16_t* rollCmd, int16_t* pitchCmd, int16_t* yawCmd)
{
  float rollCmdRaw = studentPidUpdate(&pidRollRate, rollRateDesired - rollRateActual);
  float pitchCmdRaw = studentPidUpdate(&pidPitchRate, pitchRateDesired - pitchRateActual);
  float yawCmdRaw = studentPidUpdate(&pidYawRate, yawRateDesired - yawRateActual);

  *rollCmd = saturateSignedInt16(rollCmdRaw);
  *pitchCmd = saturateSignedInt16(pitchCmdRaw);
  *yawCmd = saturateSignedInt16(yawCmdRaw);
}

void studentAttitudeControllerResetRollAttitudePID(void)
{
  studentPidReset(&pidRoll);
}

void studentAttitudeControllerResetYawAttitudePID(void)
{
  studentPidReset(&pidYaw);
}

void studentAttitudeControllerResetPitchAttitudePID(void)
{
  studentPidReset(&pidPitch);
}

void studentAttitudeControllerResetAllPID(void)
{
  studentPidReset(&pidRoll);
  studentPidReset(&pidPitch);
  studentPidReset(&pidYaw);
  studentPidReset(&pidRollRate);
  studentPidReset(&pidPitchRate);
  studentPidReset(&pidYawRate);
}



//488 TODO setup logging parameters, replace null with pointer to globabl variable

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

//488 TODO setup logging parameters, replace null with pointer to globabl variable

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

//488 TODO setup adjustment parameters, replace null with pointer to globabl variables

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

//488 TODO setup adjustment parameters, replace null with pointer to globabl variables

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
