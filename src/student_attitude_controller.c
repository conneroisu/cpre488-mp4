/**
 * student_attitude_pid_controller.c: Attitude controller using PID correctors
 */

#include "student_attitude_controller.h"

/**
 * @brief Convert float to 16 bit integer
 * Use this for converting final value to store in the control struct.
 * As later we may negate INT16_MIN, it is not used here.
 *
 * @param in float
 * @return int16_t
 */
static inline int16_t saturateSignedInt16(float in) {
  if (in > INT16_MAX) {
    return INT16_MAX;
  } else if (in < -INT16_MAX) {
    return -INT16_MAX;
  } else {
    return (int16_t)in;
  }
}

// PidObject for Roll Rate control.
// Controls the angular velocity around the roll axis (X-axis)
// Used in the inner loop of the cascaded control system
// Processes the error between desired and measured roll rate
// Outputs motor commands to achieve desired roll rate
PidObject pidRollRate;

// PidObject for Pitch Rate control.
// Controls the angular velocity around the pitch axis (Y-axis)
// Used in the inner loop of the cascaded control system
// Processes the error between desired and measured pitch rate
// Outputs motor commands to achieve desired pitch rate
PidObject pidPitchRate;

// PidObject for Yaw Rate control.
// Controls the angular velocity around the yaw axis (Z-axis)
// Used in the inner loop of the cascaded control system
// Processes the error between desired and measured yaw rate
// Outputs motor commands to achieve desired yaw rate
PidObject pidYawRate;

// PidObject for Roll Angle control.
// Controls the absolute roll angle of the drone
// Used in the outer loop of the cascaded control system
// Processes the error between desired and measured roll angle
// Outputs a desired roll rate that becomes setpoint for pidRollRate
PidObject pidRoll;

// PidObject for Pitch Angle control.
// Controls the absolute pitch angle of the drone
// Used in the outer loop of the cascaded control system
// Processes the error between desired and measured pitch angle
// Outputs a desired pitch rate that becomes setpoint for pidPitchRate
PidObject pidPitch;

// PidObject for Yaw Angle control.
// Controls the absolute yaw angle (heading) of the drone
// Used in the outer loop of the cascaded control system
// Processes the error between desired and measured yaw angle
// Outputs a desired yaw rate that becomes setpoint for pidYawRate
PidObject pidYaw;

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
void studentAttitudeControllerCorrectAttitudePID( //
    float eulerRollActual,                        //
    float eulerPitchActual,                       //
    float eulerYawActual,                         //
    float eulerRollDesired,                       //
    float eulerPitchDesired,                      //
    float eulerYawDesired,                        //
    float *rollRateDesired,                       //
    float *pitchRateDesired,                      //
    float *yawRateDesired                         //
) {

  // 488 TODO update all attitude PID's

  // 488 TODO Update PID for yaw axis, handle error update here instead of in
  // PID calculation to keep error between -180 and 180
}

/**
 * Make the controller run an update of the rate PID. Input comes from the
 * correct attitude function. The output is the actuator force.
 *
 * @param `float` **rollRateActual** the actual current roll rate
 * @param `float` **pitchRateActual** the actual current pitch rate
 * @param `float` **yawRateActual** the actual current yaw rate
 * @param `float` **rollRateDesired** the desired roll rate
 * @param `float` **pitchRateDesired** the desired pitch rate
 * @param `float` **yawRateDesired** the desired yaw rate
 * @param `int16_t` **rollCmd** the output roll command
 * @param `int16_t` **pitchCmd** the output pitch command
 * @param `int16_t` **yawCmd** the output yaw command
 */
void studentAttitudeControllerCorrectRatePID( //
    float rollRateActual,                     //
    float pitchRateActual,                    //
    float yawRateActual,                      //
    float rollRateDesired,                    //
    float pitchRateDesired,                   //
    float yawRateDesired,                     //
    int16_t *rollCmd,                         //
    int16_t *pitchCmd,                        //
    int16_t *yawCmd                           //
) {

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

/**
 *  Log variables of attitude PID controller
 */
LOG_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional output roll
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRoll.kp)
/**
 * @brief Integral output roll
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRoll.ki)
/**
 * @brief Derivative output roll
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRoll.kd)
/**
 * @brief Proportional output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.kp)
/**
 * @brief Integral output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.ki)
/**
 * @brief Derivative output pitch
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.kd)
/**
 * @brief Proportional output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYaw.kp)
/**
 * @brief Intergal output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYaw.ki)
/**
 * @brief Derivative output yaw
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYaw.kd)
LOG_GROUP_STOP(s_ &pid_attitude)

/**
 *  Log variables of attitude rate &pid controller
 */
LOG_GROUP_START(s_ &pid_rate)
/**
 * @brief Proportional output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outP, &pidRollRate.kp)
/**
 * @brief Integral output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outI, &pidRollRate.ki)
/**
 * @brief Derivative output roll rate
 */
LOG_ADD(LOG_FLOAT, roll_outD, &pidRollRate.kd)
/**
 * @brief Proportional output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.kp)
/**
 * @brief Integral output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.ki)
/**
 * @brief Derivative output pitch rate
 */
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.kd)
/**
 * @brief Proportional output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outP, &pidYawRate.kp)
/**
 * @brief Integral output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outI, &pidYawRate.ki)
/**
 * @brief Derivative output yaw rate
 */
LOG_ADD(LOG_FLOAT, yaw_outD, &pidYawRate.kd)
LOG_GROUP_STOP(s_ &pid_rate)

/**
 * Tuning settings for the gains of the PID
 * controller for the attitude of the Crazyflie which consists
 * of the Yaw Pitch and Roll
 */
PARAM_GROUP_START(s_pid_attitude)
/**
 * @brief Proportional gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &PID_ROLL_KP)
/**
 * @brief Integral gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &PID_ROLL_KI)
/**
 * @brief Derivative gain for the PID roll controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &PID_ROLL_KD)
/**
 * @brief Proportional gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &PID_PITCH_KP)
/**
 * @brief Integral gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &PID_PITCH_KI)
/**
 * @brief Derivative gain for the PID pitch controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &PID_PITCH_KD)
/**
 * @brief Proportional gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &PID_YAW_KP)
/**
 * @brief Integral gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &PID_YAW_KI)
/**
 * @brief Derivative gain for the PID yaw controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &PID_YAW_KD)
PARAM_GROUP_STOP(s_pid_attitude)

/**
 * Tuning settings for the gains of the PID controller for the rate angles of
 * the Crazyflie, which consists of the yaw, pitch and roll rates
 */
PARAM_GROUP_START(s_pid_rate)
/**
 * @brief Proportional gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kp, &PID_ROLL_RATE_KP)
/**
 * @brief Integral gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_ki, &PID_ROLL_RATE_KI)
/**
 * @brief Derivative gain for the PID roll rate controller
 */
PARAM_ADD(PARAM_FLOAT, roll_kd, &PID_ROLL_RATE_KD)
/**
 * @brief Proportional gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kp, &PID_PITCH_RATE_KP)
/**
 * @brief Integral gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_ki, &PID_PITCH_RATE_KI)
/**
 * @brief Derivative gain for the PID pitch rate controller
 */
PARAM_ADD(PARAM_FLOAT, pitch_kd, &PID_PITCH_RATE_KD)
/**
 * @brief Proportional gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kp, &PID_YAW_RATE_KP)
/**
 * @brief Integral gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_ki, &PID_YAW_RATE_KI)
/**
 * @brief Derivative gain for the PID yaw rate controller
 */
PARAM_ADD(PARAM_FLOAT, yaw_kd, &PID_YAW_RATE_KD)
PARAM_GROUP_STOP(s_pid_rate)
