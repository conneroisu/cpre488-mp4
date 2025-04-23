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

// Any rate readings less than this value are considered to be zero.
#define MEASURED_CUTOFF (float)5.0

// Store up to this amount of error readings for computing the average error.
#define ERROR_AVERAGE_MAX_READINGS 1

typedef struct average_error
{
  float error_readings[ERROR_AVERAGE_MAX_READINGS];
  int count;
  int next_write_index;
  float avg_error;
} average_error_t;

typedef struct
{
  // 488 TODO write PidObject struct
  // needs all values that will be used for PID calculations
  // error, kp, ki, kd, setpoint ...
  float kp, ki, kd, dt, setpoint, i_limit, total_error, last_d_contribution, prev_avg_error;
  int invert_error;

  average_error_t error;

  // first_error_read_saved: Indicates if the first error reading has been saved.
  // cap_error_angle: Indicates if the calculated error should be capped between -180 and 180
  int prev_error_saved, cap_error_angle;

} PidObject;

void updateAverageError(average_error_t* avg_error, float error);

float capAngle(float angle);

void studentPidInit(PidObject *pid, const float desired, const float kp,
                    const float ki, const float kd, const float dt, const int cap_error_angle, const int invert_error);

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