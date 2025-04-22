#include "student_pid.h"
#include "num.h"
#include <math.h>
#include <float.h>


const float PID_ROLL_RATE_KP = 800;
const float PID_ROLL_RATE_KI = 5;
const float PID_ROLL_RATE_KD = 60;
const float PID_PITCH_RATE_KP = 400;
const float PID_PITCH_RATE_KI = 2;
const float PID_PITCH_RATE_KD = 40;
const float PID_YAW_RATE_KP = 300;
const float PID_YAW_RATE_KI = 1;
const float PID_YAW_RATE_KD = 10;
const float PID_ROLL_KP = 10.43;
const float PID_ROLL_KI = 0.0;
const float PID_ROLL_KD = 0.0;
const float PID_PITCH_KP = 11.24;
const float PID_PITCH_KI = 10.0;
const float PID_PITCH_KD = 0.0;
const float PID_YAW_KP = 20;
const float PID_YAW_KI = 0.0;
const float PID_YAW_KD = 0.0;




/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
void studentPidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->setpoint = desired;
  pid->prev_error = 0.0f;
  pid->total_error = 0.0f;
  pid->i_limit = 0.0f;
  pid->out_limit = 0.0f;
  
  // Initialize derivative filter if enabled
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if studentPidSetError() has been used.
 * @return PID algorithm output
 */
float studentPidUpdate(PidObject* pid, const float measured, const bool updateError)
{
  float error = pid->setpoint - measured;
  float deriv = 0.0f;
  
  // Calculate derivative term
  if (pid->dt > 0.0f) {
    deriv = (error - pid->prev_error) / pid->dt;
  }
  
  // Apply derivative filter if enabled
  if (pid->enableDFilter)
  {
    deriv = lpf2pApply(&pid->dFilter, deriv);
    if (isnan(deriv)) {
      deriv = 0;
    }
  }
  
  // Update error terms if requested
  if (updateError) {
    pid->prev_error = error;
    pid->total_error += error * pid->dt;
    
    // Constrain integral term if limit is set
    if (pid->i_limit > 0.0f) {
      pid->total_error = constrain(pid->total_error, -pid->i_limit, pid->i_limit);
    }
  }
  
  // Calculate PID output
  float output = (pid->kp * error) + 
                (pid->ki * pid->total_error) + 
                (pid->kd * deriv);
  
  // Constrain output if limit is set
  if (pid->out_limit > 0.0f) {
    output = constrain(output, -pid->out_limit, pid->out_limit);
  }
  
  return output;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject* pid, const float limit) {
  pid->i_limit = limit;
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject* pid)
{
  pid->prev_error = 0.0f;
  pid->total_error = 0.0f;
}

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void studentPidSetError(PidObject* pid, const float error)
{
  pid->total_error = error;
}

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void studentPidSetDesired(PidObject* pid, const float desired)
{
  pid->setpoint = desired;
}

/**
 * Get the current desired setpoint
 * 
 * @param[in] pid  A pointer to the pid object.
 * @return The set point
 */
float studentPidGetDesired(PidObject* pid)
{
  return pid->setpoint;
}

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool studentPidIsActive(PidObject* pid)
{
  // Consider PID active if any of the gains are above a small threshold
  return (fabsf(pid->kp) > 0.001f) || 
         (fabsf(pid->ki) > 0.001f) || 
         (fabsf(pid->kd) > 0.001f);
}

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void studentPidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void studentPidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void studentPidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void studentPidSetDt(PidObject* pid, const float dt) {
  pid->dt = dt;
}
