//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// _coder_mpcmoveCodeGeneration_api.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef _CODER_MPCMOVECODEGENERATION_API_H
#define _CODER_MPCMOVECODEGENERATION_API_H

// Include files
#include "mpcmoveCodeGeneration_spec.h"
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct10_T {
  real_T Uopt[33];
  real_T Yopt[33];
  real_T Xopt[66];
  real_T Topt[11];
  real_T Slack;
  real_T Iterations;
  real_T Cost;
};

struct struct4_T {
  real_T Plant[3];
  real_T Disturbance[3];
  real_T LastMove[3];
  real_T Covariance[36];
  boolean_T iA[72];
};

struct struct6_T {
  real_T ym[3];
  real_T ref[3];
};

struct struct5_T {
  struct6_T signals;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void mpcmoveCodeGeneration(struct4_T *statedata, struct5_T *onlinedata,
                           real_T u[3], struct10_T *Info);

void mpcmoveCodeGeneration_api(const mxArray *const prhs[3], int32_T nlhs,
                               const mxArray *plhs[3]);

void mpcmoveCodeGeneration_atexit();

void mpcmoveCodeGeneration_initialize();

void mpcmoveCodeGeneration_terminate();

void mpcmoveCodeGeneration_xil_shutdown();

void mpcmoveCodeGeneration_xil_terminate();

#endif
// End of code generation (_coder_mpcmoveCodeGeneration_api.h)
