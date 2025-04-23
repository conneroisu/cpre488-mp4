/**
 * student_attitude_pid_controller.c: Attitude controller using PID correctors
 */

#include "student_attitude_controller.h"

static bool isInit = false;

// pidRollRate controls the angular velocity around the roll axis (**x-axis**).
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured roll rate
// Outputs motor commands to achieve desired roll rate
static PidObject pidRollRate = PID_OBJECT_BLANK;

// pidPitchRate controls the angular velocity around the pitch axis (Y-axis)
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured pitch rate>
// Outputs motor commands to achieve desired pitch rate
static PidObject pidPitchRate = PID_OBJECT_BLANK;

// pidYawRate controls the angular velocity around the yaw axis (Z-axis)
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured yaw rate.
// Outputs motor commands to achieve desired yaw rate
static PidObject pidYawRate = PID_OBJECT_BLANK;

// pidRoll controls the absolute roll angle of the drone (**x-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured roll angle.
// Outputs a desired roll rate that becomes setpoint for pidRollRate.
static PidObject pidRoll = PID_OBJECT_BLANK;

// pidPitch controls the absolute pitch angle of the drone (**y-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured pitch angle.
// Outputs a desired pitch rate that becomes setpoint for pidPitchRate.
static PidObject pidPitch = PID_OBJECT_BLANK;

// PidObject controls the absolute yaw angle (heading) of the drone
// (**z-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured yaw angle.
// Outputs a desired yaw rate that becomes setpoint for pidYawRate
static PidObject pidYaw = PID_OBJECT_BLANK;

/**
 * @brief Convert float to 16 bit integer
 * Use this for converting final value to store in the control struct.
 * As later we may negate INT16_MIN, it is not used here.
 *
 * @param in float
 * @return int16_t
 */
#define LIMIT 10000
static int16_t limit_control_value(float in)
{
  if (in > LIMIT)
  {
    return LIMIT;
  } 
  else if (in < -LIMIT) 
  {
    return -LIMIT;
  } 
  else 
  {
    return (int16_t)in;
  }
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

// Also reset rate PIDs, not just attitude PIDs.
void studentAttitudeControllerResetAllPID(void)
{
  studentAttitudeControllerResetRollAttitudePID();
  studentAttitudeControllerResetPitchAttitudePID();
  studentAttitudeControllerResetYawAttitudePID();
}

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
  isInit = true;

  studentPidInit(                    //
      &pidRollRate,                  //
      0,                             //
      PID_ROLL_RATE_KP,              //
      PID_ROLL_RATE_KI,              //
      PID_ROLL_RATE_KD,              //
      updateDt,
      0
  );
  studentPidInit(                    //
      &pidPitchRate,                 //
      0,                             //
      PID_PITCH_RATE_KP,             //
      PID_PITCH_RATE_KI,             //
      PID_PITCH_RATE_KD,             //
      updateDt,
      0
  );
  studentPidInit(                    //
      &pidYawRate,                   //
      0,                             //
      PID_YAW_RATE_KP,               //
      PID_YAW_RATE_KI,               //
      PID_YAW_RATE_KD,               //
      updateDt,
      0
  );

  studentPidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRAL_LIMIT);
  studentPidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRAL_LIMIT);
  studentPidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRAL_LIMIT);


  // Error is an angle for these and should
  // be capped between -180 and 180.
  studentPidInit(               //
      &pidRoll,                 //
      0,                        //
      PID_ROLL_KP,              //
      PID_ROLL_KI,              //
      PID_ROLL_KD,              //
      updateDt,
      1
  );
  studentPidInit(               //
      &pidPitch,                //
      0,                        //
      PID_PITCH_KP,             //
      PID_PITCH_KI,             //
      PID_PITCH_KD,             //
      updateDt,
      1
  );
  studentPidInit(               //
      &pidYaw,                  //
      0,                        //
      PID_YAW_KP,               //
      PID_YAW_KI,               //
      PID_YAW_KD,               //
      updateDt,
      1
  );

  studentPidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRAL_LIMIT);
  studentPidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRAL_LIMIT);
  studentPidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRAL_LIMIT);

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
 * @param `float` **eulerRollActual** - Current measured roll angle in radians
 * or degrees. Represents the drone's current rotation around its longitudinal
 * axis (X-axis).
 *
 * @param `float` **eulerPitchActual** - Current measured pitch angle in radians
 * or degrees. Represents the drone's current rotation around its lateral axis
 * (Y-axis).
 *
 * @param `float` **eulerYawActual** - Current measured yaw angle in radians or
 * degrees. Represents the drone's current rotation around its vertical axis
 * (Z-axis), determining its heading.
 *
 * @param `float` **eulerRollDesired** - Target roll angle in radians or
 * degrees. The angle that the attitude controller attempts to achieve around
 * the X-axis.
 *
 * @param `float` **eulerPitchDesired** - Target pitch angle in radians or
 * degrees. The angle that the attitude controller attempts to achieve around
 * the Y-axis.
 *
 * @param `float` **eulerYawDesired** - Target yaw angle in radians or degrees.
 * The angle that the attitude controller attempts to achieve around the Z-axis,
 * determining the desired heading.
 *
 * @param `*float` **rollRateDesired** - Pointer to store the calculated desired
 * roll rotation rate in radians/sec or degrees/sec. This value feeds into the
 * roll rate PID controller.
 *
 * @param `*float` **pitchRateDesired** - Pointer to store the calculated
 * desired pitch rotation rate in radians/sec or degrees/sec. This value feeds
 * into the pitch rate PID controller.
 *
 * @param `*float` **yawRateDesired** - Pointer to store the calculated desired
 * yaw rotation rate in radians/sec or degrees/sec. This value feeds into the
 * yaw rate PID controller.
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

  // Update setpoints
  studentPidSetDesired(&pidRoll, eulerRollDesired);
  studentPidSetDesired(&pidPitch, eulerPitchDesired);
  studentPidSetDesired(&pidYaw, eulerYawDesired);

  // Update roll and pitch PIDs
  *rollRateDesired =
      studentPidUpdate(&pidRoll, eulerRollActual, true);
  *pitchRateDesired =
      studentPidUpdate(&pidPitch, eulerPitchActual, true);

  // Update yaw PID with normalized error
  *yawRateDesired = studentPidUpdate(&pidYaw, eulerYawActual, true);
}

/**
 * Make the controller run an update of the rate PID. Input comes from the
 * correct attitude function. The output is the actuator force.
 *
 * @param `float` **rollRateActual** Measured angular velocity around the roll
 * axis (X-axis) in radians/sec or degrees/sec.
 *
 * @param `float` **pitchRateActual** Measured angular velocity around the pitch
 * axis (Y-axis) in radians/sec or degrees/sec.
 *
 * @param `float` **yawRateActual** Measured angular velocity around the yaw
 * axis (Z-axis) in radians/sec or degrees/sec.
 *
 * @param `float` **rollRateDesired** Target angular velocity around the roll
 * axis in radians/sec or degrees/sec.
 *
 * @param `float` **pitchRateDesired** Target angular velocity around the pitch
 * axis in radians/sec or degrees/sec.
 *
 * @param `float` **yawRateDesired** Target angular velocity around the yaw axis
 * in radians/sec or degrees/sec.
 *
 * @param `*int16_t` **rollCmd** Pointer to store the calculated motor command
 * value for roll control.
 *
 * @param `*int16_t` **pitchCmd** Pointer to store the calculated motor command
 * value for pitch control.
 *
 * @param `*int16_t` **yawCmd** Pointer to store the calculated motor command
 * value for yaw control.
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

  // Update setpoints
  studentPidSetDesired(&pidRollRate, rollRateDesired);
  studentPidSetDesired(&pidPitchRate, pitchRateDesired);
  studentPidSetDesired(&pidYawRate, yawRateDesired);

  // Update all attitude rate PIDs
  float rollOutput =
      studentPidUpdate(&pidRollRate, rollRateActual, true);
  float pitchOutput =
      studentPidUpdate(&pidPitchRate, pitchRateActual, true);
  float yawOutput =
      studentPidUpdate(&pidYawRate, yawRateActual, true);

  // Convert floating point outputs to int16_t motor commands
  *rollCmd = limit_control_value(rollOutput);
  *pitchCmd = limit_control_value(pitchOutput);
  *yawCmd = limit_control_value(yawOutput);
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
LOG_GROUP_STOP(s_pid_attitude)

/**
 *  Log variables of attitude rate &pid controller
 */
LOG_GROUP_START(s_pid_rate)
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
LOG_GROUP_STOP(s_pid_rate)

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