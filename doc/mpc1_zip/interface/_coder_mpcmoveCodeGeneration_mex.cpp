//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// _coder_mpcmoveCodeGeneration_mex.cpp
//
// Code generation for function 'mpcmoveCodeGeneration'
//

// Include files
#include "_coder_mpcmoveCodeGeneration_mex.h"
#include "_coder_mpcmoveCodeGeneration_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&mpcmoveCodeGeneration_atexit);
  // Module initialization.
  mpcmoveCodeGeneration_initialize();
  // Dispatch the entry-point.
  unsafe_mpcmoveCodeGeneration_mexFunction(nlhs, plhs, nrhs, prhs);
  // Module termination.
  mpcmoveCodeGeneration_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, (const char_T *)"UTF-8", true);
  return emlrtRootTLSGlobal;
}

void unsafe_mpcmoveCodeGeneration_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                              int32_T nrhs,
                                              const mxArray *prhs[3])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *outputs[3];
  int32_T b_nlhs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs < 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooFewInputsConstants", 9, 4, 21,
                        "mpcmoveCodeGeneration", 4, 21, "mpcmoveCodeGeneration",
                        4, 21, "mpcmoveCodeGeneration");
  }
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        21, "mpcmoveCodeGeneration");
  }
  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "mpcmoveCodeGeneration");
  }
  // Call the function.
  mpcmoveCodeGeneration_api(prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }
  emlrtReturnArrays(b_nlhs, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_mpcmoveCodeGeneration_mex.cpp)
