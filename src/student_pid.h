#ifndef STUDENT_PID_H_
#define STUDENT_PID_H_

#include "filter.h"
#include <stdbool.h>

// 488 TODO hard code default PID constants found from Lab_Part_1

#define PID_ROLL_RATE_KP 0.0
#define PID_ROLL_RATE_KI 0.0
#define PID_ROLL_RATE_KD 0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT 33.3

#define PID_PITCH_RATE_KP 0.0
#define PID_PITCH_RATE_KI 0.0
#define PID_PITCH_RATE_KD 0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT 33.3

#define PID_YAW_RATE_KP 0.0
#define PID_YAW_RATE_KI 0.0
#define PID_YAW_RATE_KD 0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT 166.7

#define PID_ROLL_KP 0.0
#define PID_ROLL_KI 0.0
#define PID_ROLL_KD 0.0
#define PID_ROLL_INTEGRATION_LIMIT 20.0

#define PID_PITCH_KP 0.0
#define PID_PITCH_KI 0.0
#define PID_PITCH_KD 0.0
#define PID_PITCH_INTEGRATION_LIMIT 20.0

#define PID_YAW_KP 0.0
#define PID_YAW_KI 0.0
#define PID_YAW_KD 0.0
#define PID_YAW_INTEGRATION_LIMIT 360.0

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
#define DEFAULT_PID_OUTPUT_LIMIT 0.0

typedef struct {
  float kp, ki, kd, dt, setpoint, i_limit, total_error, prev_error;
} PidObject;

void studentPidInit(PidObject *pid, const float desired, const float kp,
                    const float ki, const float kd, const float dt,
                    const float samplingRate, const float cutoffFreq,
                    bool enableDFilter);

void studentPidSetIntegralLimit(PidObject *pid, const float limit);

void studentPidReset(PidObject *pid);

float studentPidUpdate(PidObject *pid, const float measured,
                       const bool updateError);

void studentPidSetDesired(PidObject *pid, const float desired);

float studentPidGetDesired(PidObject *pid);

bool studentPidIsActive(PidObject *pid);

void studentPidSetError(PidObject *pid, const float error);

void studentPidSetKp(PidObject *pid, const float kp);

void studentPidSetKi(PidObject *pid, const float ki);

void studentPidSetKd(PidObject *pid, const float kd);

void studentPidSetDt(PidObject *pid, const float dt);
#endif /* PID_H_ */
