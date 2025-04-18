#include "student_pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

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
  pid->setpoint = 0;
  pid->total_error = 0;
  pid->total_error = 0;
  pid->i_limit = 0;
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

  // Incorporate P and D terms.
  float control = (pid->kp * error) + (pid->kd * ((error - pid->prev_error) / pid->dt));

  // Update error.
  if(updateError)
  {
    pid->prev_error = error;
    pid->total_error += error * pid->dt;

    // Limit total error.
    if(pid->total_error > pid->i_limit)
    {
      pid->total_error = pid->i_limit;
    }
  }

  // Incorporate I term.
  control += pid->ki * pid->total_error;

  return control;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject* pid, const float limit) 
{
  pid->i_limit = limit;
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject* pid)
{
  pid->prev_error = 0;
  pid->total_error = 0;
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
  return (pid->kp + pid->ki + pid->kd);
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
void studentPidSetDt(PidObject* pid, const float dt) 
{
  pid->dt = dt;
}