//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// _coder_mpcmoveCodeGeneration_mex.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef _CODER_MPCMOVECODEGENERATION_MEX_H
#define _CODER_MPCMOVECODEGENERATION_MEX_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_mpcmoveCodeGeneration_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                              int32_T nrhs,
                                              const mxArray *prhs[3]);

#endif
// End of code generation (_coder_mpcmoveCodeGeneration_mex.h)
