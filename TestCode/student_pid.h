#ifndef STUDENT_PID_H_
#define STUDENT_PID_H_

#include <stdbool.h>
#include "filter.h"

// Rate controller constants (inner loop)
extern const float PID_ROLL_RATE_KP;
extern const float PID_ROLL_RATE_KI;
extern const float PID_ROLL_RATE_KD;
#define PID_ROLL_RATE_INTEGRATION_LIMIT 33.3f
#define PID_ROLL_RATE_OUTPUT_LIMIT 500.0f

extern const float PID_PITCH_RATE_KP;
extern const float PID_PITCH_RATE_KI;
extern const float PID_PITCH_RATE_KD;
#define PID_PITCH_RATE_INTEGRATION_LIMIT 33.3f
#define PID_PITCH_RATE_OUTPUT_LIMIT 500.0f

extern const float PID_YAW_RATE_KP;
extern const float PID_YAW_RATE_KI;
extern const float PID_YAW_RATE_KD;
#define PID_YAW_RATE_INTEGRATION_LIMIT 166.7f
#define PID_YAW_RATE_OUTPUT_LIMIT 360.0f

// Attitude controller constants (outer loop)
extern const float PID_ROLL_KP;
extern const float PID_ROLL_KI;
extern const float PID_ROLL_KD;
#define PID_ROLL_INTEGRATION_LIMIT 20.0f
#define PID_ROLL_OUTPUT_LIMIT 200.0f

extern const float PID_PITCH_KP;
extern const float PID_PITCH_KI;
extern const float PID_PITCH_KD;
#define PID_PITCH_INTEGRATION_LIMIT 20.0f
#define PID_PITCH_OUTPUT_LIMIT 200.0f

extern const float PID_YAW_KP;
extern const float PID_YAW_KI;
extern const float PID_YAW_KD;
#define PID_YAW_INTEGRATION_LIMIT 360.0f
#define PID_YAW_OUTPUT_LIMIT 180.0f

typedef struct {
    // PID gains and parameters
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float dt;           // Delta time
    
    // Controller state
    float setpoint;     // Desired value
    float prev_error;   // Previous error for derivative term
    float total_error;  // Accumulated error for integral term
    
    // Limits
    float i_limit;      // Integral windup limit
    float out_limit;    // Output saturation limit
    
    // Derivative filter
    lpf2pData dFilter;  // Filter for D term
    bool enableDFilter; // Filter enable flag
} PidObject;

// Declare PID objects for all controllers
extern PidObject pidRollRate;
extern PidObject pidPitchRate;
extern PidObject pidYawRate;
extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;

// Function prototypes
void studentPidInit(PidObject *pid, const float desired, const float kp,
                   const float ki, const float kd, const float dt,
                   const float samplingRate, const float cutoffFreq,
                   bool enableDFilter);

void studentPidSetIntegralLimit(PidObject *pid, const float limit);
void studentPidSetOutputLimit(PidObject *pid, const float limit);
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
