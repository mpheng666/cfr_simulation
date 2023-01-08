//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// _coder_mpcmoveCodeGeneration_api.cpp
//
// Code generation for function 'mpcmoveCodeGeneration'
//

// Include files
#include "_coder_mpcmoveCodeGeneration_api.h"
#include "_coder_mpcmoveCodeGeneration_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131626U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "mpcmoveCodeGeneration",                              // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[36]);

static struct5_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static const mxArray *b_emlrt_marshallOut(const real_T u[33]);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[72]);

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static struct6_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *statedata,
                             const char_T *identifier, struct4_T *y);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct4_T *y);

static struct5_T emlrt_marshallIn(const emlrtStack *sp,
                                  const mxArray *onlinedata,
                                  const char_T *identifier);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3]);

static const mxArray *emlrt_marshallOut(const struct4_T *u);

static const mxArray *emlrt_marshallOut(const struct10_T *u);

static const mxArray *emlrt_marshallOut(const real_T u[3]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[36]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               boolean_T ret[72]);

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId);

// Function Definitions
static struct5_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"signals", "weights", "limits",
                                     "customconstraints"};
  emlrtMsgIdentifier thisId;
  struct5_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "signals";
  y.signals =
      d_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtCTX)sp, u, 0, 0, (const char_T *)"signals")),
                         &thisId);
  thisId.fIdentifier = "weights";
  e_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                    (const char_T *)"weights")),
                     &thisId);
  thisId.fIdentifier = "limits";
  f_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                    (const char_T *)"limits")),
                     &thisId);
  thisId.fIdentifier = "customconstraints";
  g_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                     (const char_T *)"customconstraints")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[36])
{
  f_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const real_T u[33])
{
  static const int32_T iv[2]{11, 3};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (int32_T b_i{0}; b_i < 3; b_i++) {
    for (int32_T c_i{0}; c_i < 11; c_i++) {
      pData[i + c_i] = u[c_i + 11 * b_i];
    }
    i += 11;
  }
  emlrtAssign(&y, m);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               boolean_T y[72])
{
  j_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static struct6_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[5]{"ym", "ref", "md", "mvTarget",
                                     "externalMV"};
  emlrtMsgIdentifier thisId;
  struct6_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 5,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "ym";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                  (const char_T *)"ym")),
                   &thisId, y.ym);
  thisId.fIdentifier = "ref";
  d_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                    (const char_T *)"ref")),
                     &thisId, y.ref);
  thisId.fIdentifier = "md";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                  (const char_T *)"md")),
                   &thisId);
  thisId.fIdentifier = "mvTarget";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                  (const char_T *)"mvTarget")),
                   &thisId);
  thisId.fIdentifier = "externalMV";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b(
                       (emlrtCTX)sp, u, 0, 4, (const char_T *)"externalMV")),
                   &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  g_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims{3};
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"y", "u", "du", "ecr"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "y";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                  (const char_T *)"y")),
                   &thisId);
  thisId.fIdentifier = "u";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                  (const char_T *)"u")),
                   &thisId);
  thisId.fIdentifier = "du";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                  (const char_T *)"du")),
                   &thisId);
  thisId.fIdentifier = "ecr";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                  (const char_T *)"ecr")),
                   &thisId);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *statedata,
                             const char_T *identifier, struct4_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(statedata), &thisId, y);
  emlrtDestroyArray(&statedata);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3])
{
  e_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static struct5_T emlrt_marshallIn(const emlrtStack *sp,
                                  const mxArray *onlinedata,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct5_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(onlinedata), &thisId);
  emlrtDestroyArray(&onlinedata);
  return y;
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct4_T *y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[6]{"Plant",    "Disturbance", "Noise",
                                     "LastMove", "Covariance",  "iA"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "Plant";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                  (const char_T *)"Plant")),
                   &thisId, y->Plant);
  thisId.fIdentifier = "Disturbance";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b(
                       (emlrtCTX)sp, u, 0, 1, (const char_T *)"Disturbance")),
                   &thisId, y->Disturbance);
  thisId.fIdentifier = "Noise";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                  (const char_T *)"Noise")),
                   &thisId);
  thisId.fIdentifier = "LastMove";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                  (const char_T *)"LastMove")),
                   &thisId, y->LastMove);
  thisId.fIdentifier = "Covariance";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b(
                         (emlrtCTX)sp, u, 0, 4, (const char_T *)"Covariance")),
                     &thisId, y->Covariance);
  thisId.fIdentifier = "iA";
  c_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 5,
                                                    (const char_T *)"iA")),
                     &thisId, y->iA);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId)
{
  i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static const mxArray *emlrt_marshallOut(const struct10_T *u)
{
  static const int32_T iv[2]{11, 6};
  static const int32_T i1{11};
  static const char_T *sv[7]{"Uopt",  "Yopt",       "Xopt", "Topt",
                             "Slack", "Iterations", "Cost"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Uopt",
                      b_emlrt_marshallOut(u->Uopt), 0);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Yopt",
                      b_emlrt_marshallOut(u->Yopt), 1);
  b_y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (int32_T b_i{0}; b_i < 6; b_i++) {
    for (int32_T c_i{0}; c_i < 11; c_i++) {
      pData[i + c_i] = u->Xopt[c_i + 11 * b_i];
    }
    i += 11;
  }
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Xopt", b_y, 2);
  c_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (int32_T b_i{0}; b_i < 11; b_i++) {
    pData[b_i] = u->Topt[b_i];
  }
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Topt", c_y, 3);
  d_y = nullptr;
  m = emlrtCreateDoubleScalar(u->Slack);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Slack", d_y, 4);
  e_y = nullptr;
  m = emlrtCreateDoubleScalar(u->Iterations);
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Iterations", e_y, 5);
  f_y = nullptr;
  m = emlrtCreateDoubleScalar(u->Cost);
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Cost", f_y, 6);
  return y;
}

static const mxArray *emlrt_marshallOut(const struct4_T *u)
{
  static const int32_T iv[2]{6, 6};
  static const int32_T i{3};
  static const int32_T i1{3};
  static const int32_T i2{0};
  static const int32_T i3{3};
  static const int32_T i5{72};
  static const char_T *sv[6]{"Plant",    "Disturbance", "Noise",
                             "LastMove", "Covariance",  "iA"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i4;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->Plant[0];
  pData[1] = u->Plant[1];
  pData[2] = u->Plant[2];
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Plant", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->Disturbance[0];
  pData[1] = u->Disturbance[1];
  pData[2] = u->Disturbance[2];
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Disturbance", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Noise", d_y, 2);
  e_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->LastMove[0];
  pData[1] = u->LastMove[1];
  pData[2] = u->LastMove[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"LastMove", e_y, 3);
  f_y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i4 = 0;
  for (int32_T b_i{0}; b_i < 6; b_i++) {
    for (int32_T c_i{0}; c_i < 6; c_i++) {
      pData[i4 + c_i] = u->Covariance[c_i + 6 * b_i];
    }
    i4 += 6;
  }
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"Covariance", f_y, 4);
  g_y = nullptr;
  m = emlrtCreateLogicalArray(1, &i5);
  emlrtInitLogicalArray(72, m, &u->iA[0]);
  emlrtAssign(&g_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"iA", g_y, 5);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i{0};
  static const int32_T i1{3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[36])
{
  static const int32_T dims[2]{6, 6};
  real_T(*r)[36];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[36])emlrtMxGetData(src);
  for (int32_T i{0}; i < 36; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"ymin", "ymax", "umin", "umax"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "ymin";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                  (const char_T *)"ymin")),
                   &thisId);
  thisId.fIdentifier = "ymax";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                  (const char_T *)"ymax")),
                   &thisId);
  thisId.fIdentifier = "umin";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                  (const char_T *)"umin")),
                   &thisId);
  thisId.fIdentifier = "umax";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                  (const char_T *)"umax")),
                   &thisId);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims[2]{1, 3};
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[4]{"E", "F", "G", "S"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 4,
                         (const char_T **)&fieldNames[0], 0U, (void *)&dims);
  thisId.fIdentifier = "E";
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 0,
                                                    (const char_T *)"E")),
                     &thisId);
  thisId.fIdentifier = "F";
  h_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 1,
                                                    (const char_T *)"F")),
                     &thisId);
  thisId.fIdentifier = "G";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 2,
                                                  (const char_T *)"G")),
                   &thisId);
  thisId.fIdentifier = "S";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, 0, 3,
                                                  (const char_T *)"S")),
                   &thisId);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               boolean_T ret[72])
{
  static const int32_T dims{72};
  boolean_T(*r)[72];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"logical",
                          false, 1U, (void *)&dims);
  r = (boolean_T(*)[72])emlrtMxGetLogicals(src);
  for (int32_T i{0}; i < 72; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims[2]{0, 3};
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  emlrtMxGetData(src);
  emlrtDestroyArray(&src);
}

void mpcmoveCodeGeneration_api(const mxArray *const prhs[3], int32_T nlhs,
                               const mxArray *plhs[3])
{
  static const uint32_T uv[4]{2470354077U, 154669899U, 3202593065U,
                              1509323755U};
  static const char_T *s{"configdata"};
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct10_T Info;
  struct4_T statedata;
  struct5_T onlinedata;
  real_T(*u)[3];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  u = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  // Check constant function inputs
  i = 4;
  emlrtCheckArrayChecksumR2018b(&st, prhs[0], false, &i, (const char_T **)&s,
                                &uv[0]);
  // Marshall function inputs
  emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "statedata", &statedata);
  onlinedata = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "onlinedata");
  // Invoke the target function
  mpcmoveCodeGeneration(&statedata, &onlinedata, *u, &Info);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*u);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(&statedata);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(&Info);
  }
}

void mpcmoveCodeGeneration_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  mpcmoveCodeGeneration_xil_terminate();
  mpcmoveCodeGeneration_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void mpcmoveCodeGeneration_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void mpcmoveCodeGeneration_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (_coder_mpcmoveCodeGeneration_api.cpp)
