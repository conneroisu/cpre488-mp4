#ifndef __STUDENT_PID_H
#define __STUDENT_PID_H

#include <stdbool.h>
#include "filter.h"

// Default PID gains for rate controllers
#define RATE_ROLL_KP  70.0f
#define RATE_ROLL_KI  0.0f
#define RATE_ROLL_KD  0.0f

#define RATE_PITCH_KP 70.0f
#define RATE_PITCH_KI 0.0f
#define RATE_PITCH_KD 0.0f

#define RATE_YAW_KP   50.0f
#define RATE_YAW_KI   25.0f
#define RATE_YAW_KD   0.0f

// Default PID gains for attitude controllers
#define ATT_ROLL_KP  3.5f
#define ATT_ROLL_KI  0.0f
#define ATT_ROLL_KD  0.0f

#define ATT_PITCH_KP 3.5f
#define ATT_PITCH_KI 0.0f
#define ATT_PITCH_KD 0.0f

#define ATT_YAW_KP   6.0f
#define ATT_YAW_KI   1.0f
#define ATT_YAW_KD   0.3f

// PID object structure definition
typedef struct
{
    // PID gains
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    
    // PID state variables
    float desired;      // Desired value (setpoint)
    float prevError;    // Previous error for derivative calculation
    float integ;        // Integral accumulator
    float outP;         // Proportional output
    float outI;         // Integral output
    float outD;         // Derivative output
    float output;       // Total PID output
    
    // Limits
    float integLimit;   // Integral limit
    float outputLimit;  // Output limit
    
    // Timing
    float dt;           // Delta time between updates
    
    // Filter for derivative
    bool enableDFilter; // Enable derivative low-pass filter
    lpf2pData dFilter;  // Low-pass filter for derivative term
    float deriv;        // Filtered derivative value
} PidObject;

// Function declarations
void studentPidInit(PidObject* pid, const float kp, const float ki, const float kd, const float dt,
                  const float samplingRate, const float cutoffFreq, bool enableDFilter);
float studentPidUpdate(PidObject* pid, const float error, const bool updateError);
void studentPidSetIntegralLimit(PidObject* pid, const float limit);
void studentPidReset(PidObject* pid);
void studentPidSetError(PidObject* pid, const float error);
void studentPidSetDesired(PidObject* pid, const float desired);
float studentPidGetDesired(PidObject* pid);
bool studentPidIsActive(PidObject* pid);
void studentPidSetKp(PidObject* pid, const float kp);
void studentPidSetKi(PidObject* pid, const float ki);
void studentPidSetKd(PidObject* pid, const float kd);
void studentPidSetDt(PidObject* pid, const float dt);

// Simplified initialization with default filter parameters
static inline void studentPidInit(PidObject* pid, const float kp, const float ki, const float kd, const float dt)
{
    studentPidInit(pid, kp, ki, kd, dt, 500.0f, 100.0f, false);
}

#endif // __STUDENT_PID_Hst
