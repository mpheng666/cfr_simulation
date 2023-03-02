//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// mpcmoveCodeGeneration_types.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef MPCMOVECODEGENERATION_TYPES_H
#define MPCMOVECODEGENERATION_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct10_T {
  double Uopt[33];
  double Yopt[33];
  double Xopt[66];
  double Topt[11];
  double Slack;
  double Iterations;
  double Cost;
};

struct struct4_T {
  double Plant[3];
  double Disturbance[3];
  double LastMove[3];
  double Covariance[36];
  boolean_T iA[72];
};

struct struct6_T {
  double ym[3];
  double ref[3];
};

struct struct5_T {
  struct6_T signals;
};

#endif
// End of code generation (mpcmoveCodeGeneration_types.h)
