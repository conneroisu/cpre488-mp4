#include "student_pid.h"
#include "num.h"
#include <float.h>
#include <math.h>
#include "log.h"

float PID_ROLL_RATE_KP = 60;
float PID_ROLL_RATE_KI = 1;
float PID_ROLL_RATE_KD = 5;
float PID_PITCH_RATE_KP = 60;
float PID_PITCH_RATE_KI = 1;
float PID_PITCH_RATE_KD = 5;
float PID_YAW_RATE_KP = 80;
float PID_YAW_RATE_KI = 1;
float PID_YAW_RATE_KD = 1;
float PID_ROLL_KP = 15;
float PID_ROLL_KI = 1;
float PID_ROLL_KD = 0;
float PID_PITCH_KP = 15;
float PID_PITCH_KI = 2;
float PID_PITCH_KD = 0;
float PID_YAW_KP = 2;
float PID_YAW_KI = 1;
float PID_YAW_KD = 0;

/**
 * Limit the input angle between -180 and 180
 *
 * @param angle
 * @return float
 */

 float capAngle(float angle) 
 {
   int coterminal = (int) angle / 360;
 
   angle -= coterminal * 360;
 
   // Assumed units are degrees.
   if(angle > 0)
   {
        if(angle <= 180.0f)
        {
        return angle;
        }
        else
        {
        return (-180.0f) + angle;
        }
   }
   else
   {
        if(angle >= -180.0f)
        {
        return angle;
        }
        else
        {
        return (180.0f) + angle;
        }
   }

 }

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
void studentPidInit(PidObject *pid, const float desired, const float kp,
                    const float ki, const float kd, const float dt, const int cap_error_angle, const int invert_error)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->dt = dt;
  pid->setpoint = desired;
  pid->total_error = 0;
  pid->prev_avg_error = 0;
  pid->i_limit = 0;
  pid->prev_error_saved = 0;
  pid->cap_error_angle = cap_error_angle;
  pid->invert_error = invert_error;

  pid->error.count = 0;
  pid->error.avg_error = 0;
  pid->error.next_write_index = 0;
  
  for(int i = 0; i < ERROR_AVERAGE_MAX_READINGS; ++i)
  {
    pid->error.error_readings[i] = 0;
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
float studentPidUpdate(PidObject *pid, const float measured,
                       const bool updateError) {

  // Apply measured value cutoff.
  float modified_measured = measured;

  if((float) fabs(modified_measured) < MEASURED_CUTOFF)
  {
    modified_measured = 0;
  }

  float error = (pid->invert_error) ? (modified_measured - pid->setpoint) : (pid->setpoint - modified_measured);

  if(pid->cap_error_angle)
  {
    error = capAngle(error);
  }

  updateAverageError(&pid->error, error);

  // Incorporate P term.
  float control = pid->kp * error;


  // ERROR_AVERAGE_MAX_READINGS must happen before D is updated.
  // If previous error has NOT been saved yet, don't update!.
  if(pid->prev_error_saved)
  {
    // Incorporate D term.
    // If we have read a new set of ERROR_AVERAGE_MAX_READINGS, update the D contribution.
    if(pid->error.count == ERROR_AVERAGE_MAX_READINGS && !pid->error.next_write_index)
    {
      pid->last_d_contribution = pid->kd * ((pid->error.avg_error - pid->prev_avg_error) / (pid->dt * ERROR_AVERAGE_MAX_READINGS));
    }

    control += pid->last_d_contribution;
  }

  // Update error.
  if (updateError)
  {
    // Only update prev error once we have a new set of regular error readings.
    if(pid->error.count == ERROR_AVERAGE_MAX_READINGS && !pid->error.next_write_index)
    {
      pid->prev_avg_error = pid->error.avg_error;
      pid->prev_error_saved = 1;
    }
    pid->total_error += error * pid->dt;

    // Limit total error.
    if (pid->total_error > pid->i_limit)
    {
      pid->total_error = pid->i_limit;
    } 
    else if (pid->total_error < -pid->i_limit)
    {
      pid->total_error = -pid->i_limit;
    }
  }

  // Incorporate I term.
  control += pid->ki * pid->total_error;

  return control;
}

void updateAverageError(average_error_t* avg_error, float error)
{
    // Update error average
    avg_error->error_readings[avg_error->next_write_index] = error;
    if(avg_error->next_write_index == (ERROR_AVERAGE_MAX_READINGS - 1))
    {
      avg_error->next_write_index = 0;
    }
    else
    {
      avg_error->next_write_index++;
    }

    if(avg_error->count < ERROR_AVERAGE_MAX_READINGS)
    {
      avg_error->count++;
    }

    float total = 0;

    for(int i = 0; i < avg_error->count; ++i)
    {
      total += avg_error->error_readings[i];
    }

    avg_error->avg_error = total / avg_error->count;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject *pid, const float limit) {
  pid->i_limit = limit;
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject *pid) {
  pid->prev_avg_error = 0;
  pid->total_error = 0;
  pid->prev_error_saved = 0;

  pid->error.avg_error = 0;
  pid->error.count = 0;
  pid->error.next_write_index = 0;

  for(int i = 0; i < ERROR_AVERAGE_MAX_READINGS; ++i)
  {
    pid->error.error_readings[i] = 0;
  }
}

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void studentPidSetError(PidObject *pid, const float error) {
  pid->total_error = error;
}

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void studentPidSetDesired(PidObject *pid, const float desired) {
  pid->setpoint = desired;
}

/**
 * Get the current desired setpoint
 *
 * @param[in] pid  A pointer to the pid object.
 * @return The set point
 */
float studentPidGetDesired(PidObject *pid) { return pid->setpoint; }

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool studentPidIsActive(PidObject *pid) {
  return (pid->kp + pid->ki + pid->kd) > EPSILON;
}

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void studentPidSetKp(PidObject *pid, const float kp) { pid->kp = kp; }

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void studentPidSetKi(PidObject *pid, const float ki) { pid->ki = ki; }

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void studentPidSetKd(PidObject *pid, const float kd) { pid->kd = kd; }

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void studentPidSetDt(PidObject *pid, const float dt) { pid->dt = dt; }