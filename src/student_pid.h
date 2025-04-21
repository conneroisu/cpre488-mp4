#ifndef STUDENT_PID_H_
#define STUDENT_PID_H_

#include <stdbool.h>

#define EPSILON 0.001f

extern const float PID_ROLL_RATE_KP;
extern const float PID_ROLL_RATE_KI;
extern const float PID_ROLL_RATE_KD;
#define PID_ROLL_RATE_INTEGRATION_LIMIT 33.3

extern const float PID_PITCH_RATE_KP;
extern const float PID_PITCH_RATE_KI;
extern const float PID_PITCH_RATE_KD;
#define PID_PITCH_RATE_INTEGRATION_LIMIT 33.3

extern const float PID_YAW_RATE_KP;
extern const float PID_YAW_RATE_KI;
extern const float PID_YAW_RATE_KD;
#define PID_YAW_RATE_INTEGRATION_LIMIT 166.7

extern const float PID_ROLL_KP;
extern const float PID_ROLL_KI;
extern const float PID_ROLL_KD;
#define PID_ROLL_INTEGRATION_LIMIT 20.0

extern const float PID_PITCH_KP;
extern const float PID_PITCH_KI;
extern const float PID_PITCH_KD;
#define PID_PITCH_INTEGRATION_LIMIT 20.0

extern const float PID_YAW_KP;
extern const float PID_YAW_KI;
extern const float PID_YAW_KD;
#define PID_YAW_INTEGRATION_LIMIT 360.0

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
#define DEFAULT_PID_OUTPUT_LIMIT 0.0

typedef struct
{
  // 488 TODO write PidObject struct
  // needs all values that will be used for PID calculations
  // error, kp, ki, kd, setpoint ...
  float kp, ki, kd, dt, setpoint, i_limit, total_error, prev_error;

  // first_error_read_saved: Indicates if the first error reading has been saved.
  // cap_error_angle: Indicates if the calculated error should be capped between -180 and 180
  int first_error_read_saved, cap_error_angle;

} PidObject;

// pidRollRate controls the angular velocity around the roll axis (**x-axis**).
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured roll rate
// Outputs motor commands to achieve desired roll rate
PidObject pidRollRate;

// pidPitchRate controls the angular velocity around the pitch axis (Y-axis)
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured pitch rate>
// Outputs motor commands to achieve desired pitch rate
PidObject pidPitchRate;

// pidYawRate controls the angular velocity around the yaw axis (Z-axis)
//
// Used in the inner loop of the cascaded control system.
// Processes the error between desired and measured yaw rate.
// Outputs motor commands to achieve desired yaw rate
PidObject pidYawRate;

// pidRoll controls the absolute roll angle of the drone (**x-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured roll angle.
// Outputs a desired roll rate that becomes setpoint for pidRollRate.
PidObject pidRoll;

// pidPitch controls the absolute pitch angle of the drone (**y-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured pitch angle.
// Outputs a desired pitch rate that becomes setpoint for pidPitchRate.
PidObject pidPitch;

// PidObject controls the absolute yaw angle (heading) of the drone
// (**z-axis**).
//
// Used in the outer loop of the cascaded control system.
// Processes the error between desired and measured yaw angle.
// Outputs a desired yaw rate that becomes setpoint for pidYawRate
PidObject pidYaw;

static float capAngle(float angle);

void studentPidInit(PidObject *pid, const float desired, const float kp,
                    const float ki, const float kd, const float dt, const int cap_error_angle);

void studentPidSetIntegralLimit(PidObject *pid, const float limit);

void studentPidReset(PidObject *pid);

float studentPidUpdate(PidObject *pid, const float measured, const bool updateError);

void studentPidSetDesired(PidObject *pid, const float desired);

float studentPidGetDesired(PidObject *pid);

bool studentPidIsActive(PidObject *pid);

void studentPidSetError(PidObject *pid, const float error);

void studentPidSetKp(PidObject *pid, const float kp);

void studentPidSetKi(PidObject *pid, const float ki);

void studentPidSetKd(PidObject *pid, const float kd);

void studentPidSetDt(PidObject *pid, const float dt);
#endif /* STUDENT_PID_H_ */