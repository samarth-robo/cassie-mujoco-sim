#ifndef IK_H
#define IK_H

#include "mujoco.h"

#define __IK_TASK_SPACE_TOLERANCE 0.001
#define __IK_MIN_STEP_SIZE_NORM 0.0001
#define __IK_ACCEPTABLE_JOINT_VIOLATION 0.01
#define __IK_ACCEPTABLE_EQ_CON_VIOLATION 0.001
//This is wildly large, but this is currently used for offline processing so it
//is not an issue. Rarely should the ik run this long
// #define __IK_MAX_ITER 100000 
#define __IK_MAX_ITER 1000

//
#define __IK_LOCK_ZERO_BODY_ROTATION 


//Debug
#define __IK_USEDEBUG


#ifdef __IK_USEDEBUG
#define DEBUG_MSG(str) do { std::cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

bool cassie_ik( void* m_ptr, void* d_ptr, 
                double lx, double ly, double lz, 
                double rx, double ry, double rz, 
                double comx, double comy, double comz,
                bool zero_hip_yaw);

double* cassie_task_space_vel( void* m_ptr, void* d_ptr, double ldx, double ldy, double ldz,
               double rdx, double rdy, double rdz,
               double comdx, double comdy, double comdz);
// void ik(double x, double y, double z);

#endif // IK_H
