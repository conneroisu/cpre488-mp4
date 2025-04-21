#include "stabilizer.h"
#include "stabilizer_types.h"
#include "student_attitude_controller.h"
#include "sensfusion6.h"
#include "controller_student.h"
#include "log.h"
#include "debug.h"
#include "param.h"
#include "math3d.h"

#define STUDENT_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float thrustDesired;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerStudentInit(void)
{
  studentAttitudeControllerInit(STUDENT_UPDATE_DT);
}

bool controllerStudentTest(void)
{
  bool pass = true;
  pass &= studentAttitudeControllerTest();
  return pass;
}

static float capAngle(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void controllerStudent(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
  {
    if(setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable){
      DEBUG_PRINT("Student controller does not support position or velocity mode.");
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      return;
    }

    // Handle yaw mixed control mode (rate mode for yaw)
    if (setpoint->mode.yaw == modeVelocity)
    {
      attitudeDesired.yaw += setpoint->attitudeRate.z * STUDENT_UPDATE_DT;
      attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    }
    else
    {
      attitudeDesired.yaw = setpoint->attitude.yaw;
    }

    attitudeDesired.roll = setpoint->attitude.roll;
    attitudeDesired.pitch = setpoint->attitude.pitch;

    thrustDesired = setpoint->thrust;

    studentAttitudeControllerCorrectAttitudePID(
      state->attitude.roll,
      state->attitude.pitch,
      state->attitude.yaw,
      attitudeDesired.roll,
      attitudeDesired.pitch,
      attitudeDesired.yaw,
      &rateDesired.roll,
      &rateDesired.pitch,
      &rateDesired.yaw
    );

    // If setpoint mode is velocity, overwrite rateDesired
    if (setpoint->mode.roll == modeVelocity)
    {
      rateDesired.roll = setpoint->attitudeRate.roll;
      studentAttitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity)
    {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      studentAttitudeControllerResetPitchAttitudePID();
    }
    if (setpoint->mode.yaw == modeVelocity)
    {
      rateDesired.yaw = setpoint->attitudeRate.yaw;
      studentAttitudeControllerResetYawAttitudePID();
    }

    // Run the rate PID controller
    int16_t rollCmd, pitchCmd, yawCmd;
    studentAttitudeControllerCorrectRatePID(
      sensors->gyro.x,
      sensors->gyro.y,
      sensors->gyro.z,
      rateDesired.roll,
      rateDesired.pitch,
      rateDesired.yaw,
      &rollCmd,
      &pitchCmd,
      &yawCmd
    );

    // Set control outputs
    control->thrust = (uint16_t)constrain(thrustDesired, 0, 60000);
    control->roll = rollCmd;
    control->pitch = pitchCmd;
    control->yaw = yawCmd;

    // If no thrust, zero all outputs and reset
    if (thrustDesired <= 0.01f)
    {
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      control->thrust = 0;
      studentAttitudeControllerResetAllPID();
    }

    // Save for logging
    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = sensors->gyro.x;
    r_pitch = sensors->gyro.y;
    r_yaw = sensors->gyro.z;
    accelz = sensors->acc.z;
  }
}

LOG_GROUP_START(ctrlStdnt)
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &accelz)
LOG_ADD(LOG_FLOAT, roll, &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch, &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw, &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate, &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate, &rateDesired.yaw)
LOG_GROUP_STOP(ctrlStdnt)

PARAM_GROUP_START(ctrlStdnt)
PARAM_ADD(PARAM_FLOAT, placeHolder, NULL)
PARAM_GROUP_STOP(ctrlStdnt)
