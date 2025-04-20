
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "controller_student.h"
#include "sensfusion6.h"
#include "student_attitude_controller.h"

#include "debug.h"
#include "log.h"

#include "math3d.h"
#include "param.h"

// delta time between calls to the update function
#define STUDENT_UPDATE_DT (float)(1.0f / ATTITUDE_RATE)

// desired vehicle state as calculated by PID controllers
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float thrustDesired;

// variables used only for logging PID command outputs
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerStudentInit(void) {
  studentAttitudeControllerInit(STUDENT_UPDATE_DT);
}

bool controllerStudentTest(void) {
  bool pass = true;
  // controller passes check if attitude controller passes
  pass &= studentAttitudeControllerTest();

  return pass;
}

/**
 * Limit the input angle between -180 and 180
 *
 * @param angle
 * @return float
 */

static float capAngle(float angle) 
{
  int coterminal = (int) angle / 360;

  angle -= coterminal * 360;

  // Assumed units are degrees.
  if(angle <= 180.0f)
  {
    return angle;
  }
  else
  {
    return (-180.0f) + angle;
  }
}

/**
 * This function is called periodically to update the PID loop,
 * Reads state estimate and setpoint values and passes them
 * to the functions that perform PID calculations,
 * attitude PID and attitude rate PID
 *
 * @param control Output, struct is modified as the ouput of the control loop
 * @param setpoint Input, setpoints for thrust, attitude, position, velocity
 * etc. of the quad
 * @param sensors Input, Raw sensor values (typically want to use the state
 * estimated instead) includes gyro, accelerometer, barometer, magnatometer
 * @param state Input, A more robust way to measure the current state of the
 * quad, allows for direct measurements of the orientation of the quad. Includes
 * attitude, position, velocity, and acceleration
 * @param tick Input, system clock
 */
void controllerStudent(control_t *control, setpoint_t *setpoint,
                       const sensorData_t *sensors, const state_t *state,
                       const uint32_t tick) {

  float p_rate, r_rate, y_rate = 0;


  // check if time to update the attutide controller
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {

    //only support attitude and attitude rate control
    if(setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable)
    {
      DEBUG_PRINT("Student controller does not support vehicle position or velocity mode. Check flight mode.");
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      return;
    }
    else
    {
      // When in attitude control, attitude rates are calculated via the attitude PID controller.
      // When in attitude rate control, attitude rates are not calculated and instead directly passed.
      // When in mixed, roll and pitch attitudes are set (so the rates are calculated), but the yaw rate is passed in directly.

      // Attitude control (yaw can be abs or velocity since the attitude PID needs to run for "attitude" and "mixed" modes)
      if(setpoint->mode.pitch == modeAbs && setpoint->mode.roll == modeAbs && setpoint->mode.yaw != modeDisable)
      {
        // Run attitude PID to get the rates.
        studentAttitudeControllerCorrectAttitudePID
        (
          state->attitude.roll,
          state->attitude.pitch,
          state->attitude.yaw,
          setpoint->attitude.roll,
          setpoint->attitude.pitch,
          setpoint->attitude.yaw,
          &r_rate,
          &p_rate,
          &y_rate
        );

        // Update desired attitudes
        attitudeDesired = setpoint->attitude;

        // If mixed mode, overwrite yaw to the setpoint yaw rate.
        if(setpoint->mode.yaw == modeVelocity)
        {
          y_rate = setpoint->attitudeRate.yaw;

          // Yaw attitude PID should be reset since we are not using its value.
          studentAttitudeControllerResetYawAttitudePID();
        }
      }
      // Attitude rate control
      else if(setpoint->mode.pitch == modeVelocity && setpoint->mode.roll == modeVelocity && setpoint->mode.yaw == modeVelocity)
      {
        r_rate = setpoint->attitudeRate.roll;
        p_rate = setpoint->attitudeRate.pitch;
        y_rate = setpoint->attitudeRate.yaw;

        // Reset all attitude PIDs to avoid error build up.
        // We are not using this PID loop currently.
        studentAttitudeControllerResetAllPID();
      }

      // Run the rate PID to get control values.
      // Roll, pitch, and yaw values set.
      studentAttitudeControllerCorrectRatePID
      (
        sensors->gyro.x,
        sensors->gyro.y,
        sensors->gyro.z,
        r_rate,
        p_rate,
        y_rate,
        &control->roll,
        &control->pitch,
        &control->yaw
      );

      // Update desired rates.
      rateDesired.pitch = p_rate;
      rateDesired.roll = r_rate;
      rateDesired.yaw = y_rate;
    }

  }

  // Set thrust
  control->thrust = setpoint->thrust;

  // Reset PID if no thrust and set control values to 0.
  if(!setpoint->thrust)
  {
    control->pitch = 0;
    control->roll = 0;
    control->yaw = 0;
    studentAttitudeControllerResetAllPID();
  }

  //copy values for logging
  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;
  r_roll = sensors->gyro.x;
  r_pitch = sensors->gyro.y;
  r_yaw = sensors->gyro.z;
  accelz = sensors->acc.z;
}
