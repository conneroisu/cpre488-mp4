#include "student_pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
void studentPidInit(PidObject* pid, const float kp, const float ki,
             const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  // Initialize all the values in the PidObject struct
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  
  pid->desired = 0.0f;
  pid->prevError = 0.0f;
  pid->integ = 0.0f;
  pid->outP = 0.0f;
  pid->outI = 0.0f;
  pid->outD = 0.0f;
  pid->output = 0.0f;
  
  pid->integLimit = 0.0f;  // No limit by default
  pid->outputLimit = 0.0f; // No limit by default

  // Additional initialization for optional low pass filter
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
 * @param[in] error       The current error (or measured value if updateError is TRUE)
 * @param[in] updateError Set to TRUE if error should be calculated internally (measured is passed).
 *                        Set to FALSE if studentPidSetError() has been used (error is passed directly).
 * @return PID algorithm output
 */
float studentPidUpdate(PidObject* pid, const float error, const bool updateError)
{
    float currentError;
    
    // Calculate error if needed
    if (updateError) {
        // If updateError is true, 'error' is actually the measured value
        currentError = pid->desired - error;
    } else {
        // If updateError is false, 'error' is the pre-calculated error
        currentError = error;
    }
    
    // Calculate P term
    pid->outP = pid->kp * currentError;
    
    // Calculate I term
    pid->integ += currentError * pid->dt;
    
    // Constrain the integral (unless the integral limit is zero)
    if (pid->integLimit > 0) {
        pid->integ = constrain(pid->integ, -pid->integLimit, pid->integLimit);
    }
    
    pid->outI = pid->ki * pid->integ;
    
    // Calculate D term
    float deriv = (currentError - pid->prevError) / pid->dt;
    pid->prevError = currentError;
    
    // Optionally enable derivative low pass filtering
    if (pid->enableDFilter)
    {
        pid->deriv = lpf2pApply(&pid->dFilter, deriv);
        if (isnan(pid->deriv)) {
            pid->deriv = 0;
        }
    } else {
        pid->deriv = deriv;
    }
    
    pid->outD = pid->kd * pid->deriv;
    
    // Sum the PID output
    pid->output = pid->outP + pid->outI + pid->outD;
    
    // Constrain the total PID output (unless the output Limit is zero)
    if (pid->outputLimit > 0) {
        pid->output = constrain(pid->output, -pid->outputLimit, pid->outputLimit);
    }
    
    return pid->output;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->integLimit = limit;
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject* pid)
{
    pid->prevError = 0;
    pid->integ = 0;
    pid->outP = 0;
    pid->outI = 0;
    pid->outD = 0;
    pid->output = 0;
}

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void studentPidSetError(PidObject* pid, const float error)
{
    pid->prevError = error;
}

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] desired The new set point
 */
void studentPidSetDesired(PidObject* pid, const float desired)
{
    pid->desired = desired;
}

/**
 * Get the current desired setpoint
 * 
 * @param[in] pid  A pointer to the pid object.
 * @return The set point
 */
float studentPidGetDesired(PidObject* pid)
{
    return pid->desired;
}

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool studentPidIsActive(PidObject* pid)
{
    // PID is active if at least one of the gains is above a small threshold
    const float epsilon = 1e-6f; // Small threshold to avoid floating point errors
    
    return (fabsf(pid->kp) > epsilon || 
            fabsf(pid->ki) > epsilon || 
            fabsf(pid->kd) > epsilon);
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
