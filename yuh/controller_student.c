
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "student_attitude_controller.h"
#include "sensfusion6.h"
#include "controller_student.h"

#include "log.h"
#include "debug.h"

#include "param.h"
#include "math3d.h"

//delta time between calls to the update function
#define STUDENT_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

//desired vehicle state as calculated by PID controllers
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float thrustDesired;

//variables used only for logging PID command outputs
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
  //controller passes check if attitude controller passes
  pass &= studentAttitudeControllerTest();

  return pass;
}


/**
 * Limit the input angle between -180 and 180
 * 
 * @param angle 
 * @return float 
 */
static float capAngle(float angle) {
  //488 TODO
  return 0;
}


/**
 * This function is called periodically to update the PID loop,
 * Reads state estimate and setpoint values and passes them
 * to the functions that perform PID calculations,
 * attitude PID and attitude rate PID
 * 
 * @param control Output, struct is modified as the ouput of the control loop
 * @param setpoint Input, setpoints for thrust, attitude, position, velocity etc. of the quad
 * @param sensors Input, Raw sensor values (typically want to use the state estimated instead) includes gyro, 
 * accelerometer, barometer, magnatometer 
 * @param state Input, A more robust way to measure the current state of the quad, allows for direct
 * measurements of the orientation of the quad. Includes attitude, position, velocity,
 * and acceleration
 * @param tick Input, system clock
 */
void controllerStudent(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{

  //488 TODO, write main controller function

  // check if time to update the attutide controller
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

    //only support attitude and attitude rate control
    if(setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable){
      DEBUG_PRINT("Student controller does not support vehicle position or velocity mode. Check flight mode.");
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      return;
    }

    // 488 TODO if yaw is in rate mode, move the yaw angle setpoint accordingly

    
    // 488 TODO set desired attitude, roll, pitch, and yaw angles


    // 488 TODO set desired thrust



    // 488 TODO Run the attitude controller update with the actual attitude and desired attitude
    // outputs the desired attitude rates


    // 488 TODO if velocity mode, overwrite rateDesired output
    // from the attitude controller with the setpoint value
    // Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later

    

    // 488 TODO update the attitude rate PID, given the current angular rate 
    // read by the gyro and the desired rate 


  }

  //488 TODO set control->thrust 

  //488 TODO if no thrust active, set all outputs to 0 and reset PID variables


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

/**
 * Logging variables for the command and reference signals for the
 * student PID controller
 */
LOG_GROUP_START(ctrlStdnt)

// 488 TODO setup logging parameters, replace null with pointer to globabl variable

/**
 * @brief Thrust command output
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, NULL)
/**
 * @brief Roll command output
 */
LOG_ADD(LOG_FLOAT, cmd_roll, NULL)
/**
 * @brief Pitch command output
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, NULL)
/**
 * @brief yaw command output
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, NULL)
/**
 * @brief Gyro roll measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_roll, NULL)
/**
 * @brief Gyro pitch measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_pitch, NULL)
/**
 * @brief Gyro yaw rate measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_yaw, NULL)
/**
 * @brief Acceleration in the z axis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, NULL)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, NULL)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, NULL)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw, NULL)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, NULL)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, NULL)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, NULL)

LOG_GROUP_STOP(ctrlStdnt)


/**
 * Controller parameters
 */
PARAM_GROUP_START(ctrlStdnt)

//488 TODO optionally add any parameters to modify the controller code while running

PARAM_ADD(PARAM_FLOAT, placeHolder, NULL)

PARAM_GROUP_STOP(ctrlStdnt)