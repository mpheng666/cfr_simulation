//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// qpkwik.cpp
//
// Code generation for function 'qpkwik'
//

// Include files
#include "qpkwik.h"
#include "norm.h"
#include "xgerc.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <math.h>

// Function Declarations
namespace coder {
static void DropConstraint(int kDrop, boolean_T iA[72], int *nA, int iC[72]);

static double KWIKfactor(const double Ac[504], const int iC[72], int nA,
                         const double Linv[49], double RLinv[49], double D[49],
                         double H[49]);

} // namespace coder
static int div_nde_s32_floor(int numerator);

static double rt_hypotd_snf(double u0, double u1);

// Function Definitions
namespace coder {
static void DropConstraint(int kDrop, boolean_T iA[72], int *nA, int iC[72])
{
  if (kDrop > 0) {
    int q0;
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      q0 = *nA;
      if (q0 < -2147483647) {
        q0 = MIN_int32_T;
      } else {
        q0--;
      }
      for (int i{kDrop}; i <= q0; i++) {
        iC[i - 1] = iC[i];
      }
    }
    iC[*nA - 1] = 0;
    q0 = *nA;
    if (q0 < -2147483647) {
      *nA = MIN_int32_T;
    } else {
      *nA = q0 - 1;
    }
  }
}

static double KWIKfactor(const double Ac[504], const int iC[72], int nA,
                         const double Linv[49], double RLinv[49], double D[49],
                         double H[49])
{
  double Q[49];
  double R[49];
  double TL[49];
  double tau[7];
  double work[7];
  double Status;
  double c;
  int b_i;
  int exitg1;
  int i;
  int i1;
  int ia;
  int ii;
  int ix0;
  int knt;
  int lastc;
  int lastv;
  boolean_T exitg2;
  Status = 1.0;
  std::memset(&RLinv[0], 0, 49U * sizeof(double));
  for (i = 0; i < nA; i++) {
    for (b_i = 0; b_i < 7; b_i++) {
      i1 = b_i + 7 * i;
      RLinv[i1] = 0.0;
      for (knt = 0; knt < 7; knt++) {
        RLinv[i1] += Linv[b_i + 7 * knt] * Ac[(iC[i] + 72 * knt) - 1];
      }
    }
  }
  std::copy(&RLinv[0], &RLinv[49], &TL[0]);
  for (i = 0; i < 7; i++) {
    tau[i] = 0.0;
    work[i] = 0.0;
  }
  for (i = 0; i < 7; i++) {
    ii = i * 7 + i;
    if (i + 1 < 7) {
      double atmp;
      atmp = TL[ii];
      ix0 = ii + 2;
      tau[i] = 0.0;
      c = internal::blas::xnrm2(6 - i, TL, ii + 2);
      if (c != 0.0) {
        double beta1;
        double d;
        d = TL[ii];
        beta1 = rt_hypotd_snf(d, c);
        if (d >= 0.0) {
          beta1 = -beta1;
        }
        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          b_i = (ii - i) + 7;
          do {
            knt++;
            for (lastc = ix0; lastc <= b_i; lastc++) {
              TL[lastc - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
          beta1 = rt_hypotd_snf(atmp, internal::blas::xnrm2(6 - i, TL, ii + 2));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }
          tau[i] = (beta1 - atmp) / beta1;
          c = 1.0 / (atmp - beta1);
          for (lastc = ix0; lastc <= b_i; lastc++) {
            TL[lastc - 1] *= c;
          }
          for (lastc = 0; lastc < knt; lastc++) {
            beta1 *= 1.0020841800044864E-292;
          }
          atmp = beta1;
        } else {
          tau[i] = (beta1 - d) / beta1;
          c = 1.0 / (d - beta1);
          b_i = (ii - i) + 7;
          for (lastc = ix0; lastc <= b_i; lastc++) {
            TL[lastc - 1] *= c;
          }
          atmp = beta1;
        }
      }
      TL[ii] = 1.0;
      if (tau[i] != 0.0) {
        lastv = 7 - i;
        knt = (ii - i) + 6;
        while ((lastv > 0) && (TL[knt] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 6 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          knt = (ii + (lastc - 1) * 7) + 7;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia + 1 <= knt + lastv) {
              if (TL[ia] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        ix0 = ii + 8;
        if (lastc != 0) {
          std::memset(&work[0], 0, lastc * sizeof(double));
          b_i = (ii + 7 * (lastc - 1)) + 8;
          for (int iac{ix0}; iac <= b_i; iac += 7) {
            c = 0.0;
            i1 = (iac + lastv) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += TL[ia - 1] * TL[(ii + ia) - iac];
            }
            knt = div_nde_s32_floor((iac - ii) - 8);
            work[knt] += c;
          }
        }
        internal::blas::xgerc(lastv, lastc, -tau[i], ii + 1, work, TL, ii + 8);
      }
      TL[ii] = atmp;
    } else {
      tau[6] = 0.0;
    }
  }
  for (ix0 = 0; ix0 < 7; ix0++) {
    for (i = 0; i <= ix0; i++) {
      knt = i + 7 * ix0;
      R[knt] = TL[knt];
    }
    b_i = ix0 + 2;
    if (b_i <= 7) {
      std::memset(&R[(ix0 * 7 + b_i) + -1], 0, (-b_i + 8) * sizeof(double));
    }
    work[ix0] = 0.0;
  }
  for (i = 6; i >= 0; i--) {
    ii = (i + i * 7) + 8;
    if (i + 1 < 7) {
      TL[ii - 8] = 1.0;
      if (tau[i] != 0.0) {
        lastv = 7 - i;
        knt = ii - i;
        while ((lastv > 0) && (TL[knt - 2] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 6 - i;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          knt = ii + (lastc - 1) * 7;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia <= (knt + lastv) - 1) {
              if (TL[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia++;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        if (lastc != 0) {
          std::memset(&work[0], 0, lastc * sizeof(double));
          b_i = ii + 7 * (lastc - 1);
          for (int iac{ii}; iac <= b_i; iac += 7) {
            c = 0.0;
            i1 = (iac + lastv) - 1;
            for (ia = iac; ia <= i1; ia++) {
              c += TL[ia - 1] * TL[((ii + ia) - iac) - 8];
            }
            knt = div_nde_s32_floor(iac - ii);
            work[knt] += c;
          }
        }
        internal::blas::xgerc(lastv, lastc, -tau[i], ii - 7, work, TL, ii);
      }
      ix0 = ii - 6;
      b_i = (ii - i) - 1;
      for (lastc = ix0; lastc <= b_i; lastc++) {
        TL[lastc - 1] *= -tau[i];
      }
    }
    TL[ii - 8] = 1.0 - tau[i];
    for (ix0 = 0; ix0 < i; ix0++) {
      TL[(ii - ix0) - 9] = 0.0;
    }
  }
  for (ix0 = 0; ix0 < 7; ix0++) {
    for (i = 0; i < 7; i++) {
      knt = i + 7 * ix0;
      Q[knt] = TL[knt];
    }
  }
  i = 0;
  do {
    exitg1 = 0;
    if (i <= nA - 1) {
      if (std::abs(R[i + 7 * i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      for (i = 0; i < 7; i++) {
        for (ix0 = 0; ix0 < 7; ix0++) {
          c = 0.0;
          for (b_i = 0; b_i < 7; b_i++) {
            c += Linv[b_i + 7 * i] * Q[b_i + 7 * ix0];
          }
          TL[i + 7 * ix0] = c;
        }
      }
      std::memset(&RLinv[0], 0, 49U * sizeof(double));
      for (ix0 = nA; ix0 >= 1; ix0--) {
        b_i = 7 * (ix0 - 1);
        i1 = (ix0 + b_i) - 1;
        RLinv[i1] = 1.0;
        for (lastc = ix0; lastc <= nA; lastc++) {
          knt = (ix0 + 7 * (lastc - 1)) - 1;
          RLinv[knt] /= R[i1];
        }
        if (ix0 > 1) {
          for (i = 0; i <= ix0 - 2; i++) {
            for (lastc = ix0; lastc <= nA; lastc++) {
              i1 = 7 * (lastc - 1);
              knt = i + i1;
              RLinv[knt] -= R[i + b_i] * RLinv[(ix0 + i1) - 1];
            }
          }
        }
      }
      if (nA > 2147483646) {
        knt = MAX_int32_T;
      } else {
        knt = nA + 1;
      }
      for (i = 0; i < 7; i++) {
        for (ix0 = i + 1; ix0 < 8; ix0++) {
          b_i = i + 7 * (ix0 - 1);
          H[b_i] = 0.0;
          for (lastc = knt; lastc < 8; lastc++) {
            i1 = 7 * (lastc - 1);
            H[b_i] -= TL[i + i1] * TL[(ix0 + i1) - 1];
          }
          H[(ix0 + 7 * i) - 1] = H[b_i];
        }
      }
      for (ix0 = 0; ix0 < nA; ix0++) {
        for (i = 0; i < 7; i++) {
          b_i = i + 7 * ix0;
          D[b_i] = 0.0;
          for (lastc = ix0 + 1; lastc <= nA; lastc++) {
            i1 = 7 * (lastc - 1);
            D[b_i] += TL[i + i1] * RLinv[ix0 + i1];
          }
        }
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return Status;
}

} // namespace coder
static int div_nde_s32_floor(int numerator)
{
  int b_numerator;
  if ((numerator < 0) && (numerator % 7 != 0)) {
    b_numerator = -1;
  } else {
    b_numerator = 0;
  }
  return numerator / 7 + b_numerator;
}

static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

namespace coder {
void qpkwik(const double Linv[49], const double Hinv[49], const double f[7],
            const double Ac[504], const double b[72], boolean_T iA[72],
            double x[7], double lambda[72], int *status)
{
  double cTol[72];
  double D[49];
  double H[49];
  double RLinv[49];
  double U[49];
  double Opt[14];
  double Rhs[14];
  double r[7];
  double z[7];
  double cVal;
  double rMin;
  int iC[72];
  int b_exponent;
  int b_i;
  int exponent;
  int i;
  int idx;
  int k;
  int kDrop;
  int nA;
  int tmp;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T guard1{false};
  for (i = 0; i < 7; i++) {
    x[i] = 0.0;
  }
  std::memset(&lambda[0], 0, 72U * sizeof(double));
  *status = 1;
  for (i = 0; i < 7; i++) {
    r[i] = 0.0;
  }
  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 72; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }
  nA = 0;
  for (i = 0; i < 72; i++) {
    if (iA[i]) {
      nA++;
      iC[nA - 1] = i + 1;
    }
  }
  guard1 = false;
  if (nA > 0) {
    int exitg3;
    std::memset(&Opt[0], 0, 14U * sizeof(double));
    for (i = 0; i < 7; i++) {
      Rhs[i] = f[i];
      Rhs[i + 7] = 0.0;
    }
    DualFeasible = false;
    tmp = static_cast<int>(std::round(0.3 * static_cast<double>(nA)));
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= 316)) {
        cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
        if (cVal < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            std::memset(&iA[0], 0, 72U * sizeof(boolean_T));
            std::memset(&iC[0], 0, 72U * sizeof(int));
            ColdReset = true;
          }
        } else {
          for (kDrop = 0; kDrop < nA; kDrop++) {
            if (kDrop + 1 > 2147483640) {
              idx = MAX_int32_T;
            } else {
              idx = kDrop + 8;
            }
            Rhs[idx - 1] = b[iC[kDrop] - 1];
            for (i = kDrop + 1; i <= nA; i++) {
              idx = (i + 7 * kDrop) - 1;
              U[idx] = 0.0;
              for (k = 0; k < nA; k++) {
                U[idx] += RLinv[(i + 7 * k) - 1] * RLinv[kDrop + 7 * k];
              }
              U[kDrop + 7 * (i - 1)] = U[idx];
            }
          }
          for (i = 0; i < 7; i++) {
            cVal = 0.0;
            for (b_i = 0; b_i < 7; b_i++) {
              cVal += H[i + 7 * b_i] * Rhs[b_i];
            }
            Opt[i] = cVal;
            for (k = 0; k < nA; k++) {
              Opt[i] += D[i + 7 * k] * Rhs[k + 7];
            }
          }
          for (i = 0; i < nA; i++) {
            cVal = 0.0;
            for (b_i = 0; b_i < 7; b_i++) {
              cVal += D[b_i + 7 * i] * Rhs[b_i];
            }
            Opt[i + 7] = cVal;
            for (k = 0; k < nA; k++) {
              Opt[i + 7] += U[i + 7 * k] * Rhs[k + 7];
            }
          }
          cVal = -1.0E-12;
          kDrop = 0;
          for (i = 0; i < nA; i++) {
            if (i + 1 > 2147483640) {
              idx = MAX_int32_T;
            } else {
              idx = i + 8;
            }
            lambda[iC[i] - 1] = Opt[idx - 1];
            if (i + 1 > 2147483640) {
              idx = MAX_int32_T;
            } else {
              idx = i + 8;
            }
            if ((Opt[idx - 1] < cVal) && (i + 1 <= nA)) {
              kDrop = i + 1;
              if (i + 1 > 2147483640) {
                idx = MAX_int32_T;
              } else {
                idx = i + 8;
              }
              cVal = Opt[idx - 1];
            }
          }
          if (kDrop <= 0) {
            DualFeasible = true;
            for (i = 0; i < 7; i++) {
              x[i] = Opt[i];
            }
          } else {
            (*status)++;
            if (tmp <= 5) {
              i = 5;
            } else {
              i = tmp;
            }
            if (*status > i) {
              nA = 0;
              std::memset(&iA[0], 0, 72U * sizeof(boolean_T));
              std::memset(&iC[0], 0, 72U * sizeof(int));
              ColdReset = true;
            } else {
              lambda[iC[kDrop - 1] - 1] = 0.0;
              DropConstraint(kDrop, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          std::memset(&lambda[0], 0, 72U * sizeof(double));
          for (i = 0; i < 7; i++) {
            cVal = 0.0;
            for (b_i = 0; b_i < 7; b_i++) {
              cVal += -Hinv[i + 7 * b_i] * f[b_i];
            }
            x[i] = cVal;
          }
        }
        exitg3 = 1;
      }
    } while (exitg3 == 0);
    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (i = 0; i < 7; i++) {
      cVal = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        cVal += -Hinv[i + 7 * b_i] * f[b_i];
      }
      x[i] = cVal;
    }
    guard1 = true;
  }
  if (guard1) {
    double Xnorm0;
    boolean_T exitg2;
    Xnorm0 = b_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= 316)) {
      double cMin;
      double t;
      boolean_T exitg4;
      cMin = -1.0E-6;
      tmp = -1;
      for (i = 0; i < 72; i++) {
        if (!cTolComputed) {
          for (k = 0; k < 7; k++) {
            z[k] = std::abs(Ac[i + 72 * k] * x[k]);
          }
          if (!std::isnan(z[0])) {
            idx = 1;
          } else {
            idx = 0;
            k = 2;
            exitg4 = false;
            while ((!exitg4) && (k < 8)) {
              if (!std::isnan(z[k - 1])) {
                idx = k;
                exitg4 = true;
              } else {
                k++;
              }
            }
          }
          if (idx == 0) {
            cVal = z[0];
          } else {
            cVal = z[idx - 1];
            b_i = idx + 1;
            for (k = b_i; k < 8; k++) {
              t = z[k - 1];
              if (cVal < t) {
                cVal = t;
              }
            }
          }
          cTol[i] = std::fmax(cTol[i], cVal);
        }
        if (!iA[i]) {
          t = 0.0;
          for (b_i = 0; b_i < 7; b_i++) {
            t += Ac[i + 72 * b_i] * x[b_i];
          }
          cVal = (t - b[i]) / cTol[i];
          if (cVal < cMin) {
            cMin = cVal;
            tmp = i;
          }
        }
      }
      cTolComputed = true;
      if (tmp + 1 <= 0) {
        exitg2 = true;
      } else if (*status == 316) {
        *status = 0;
        exitg2 = true;
      } else {
        int exitg1;
        do {
          exitg1 = 0;
          if ((tmp + 1 > 0) && (*status <= 316)) {
            boolean_T guard2{false};
            guard2 = false;
            if (nA == 0) {
              for (b_i = 0; b_i < 7; b_i++) {
                t = 0.0;
                for (idx = 0; idx < 7; idx++) {
                  t += Hinv[b_i + 7 * idx] * Ac[tmp + 72 * idx];
                }
                z[b_i] = t;
              }
              guard2 = true;
            } else {
              cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
              if (cVal <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (b_i = 0; b_i < 49; b_i++) {
                  U[b_i] = -H[b_i];
                }
                for (b_i = 0; b_i < 7; b_i++) {
                  t = 0.0;
                  for (idx = 0; idx < 7; idx++) {
                    t += U[b_i + 7 * idx] * Ac[tmp + 72 * idx];
                  }
                  z[b_i] = t;
                }
                for (i = 0; i < nA; i++) {
                  t = 0.0;
                  for (b_i = 0; b_i < 7; b_i++) {
                    t += Ac[tmp + 72 * b_i] * D[b_i + 7 * i];
                  }
                  r[i] = t;
                }
                guard2 = true;
              }
            }
            if (guard2) {
              kDrop = 0;
              cMin = 0.0;
              ColdReset = true;
              DualFeasible = true;
              if (nA > 0) {
                idx = 0;
                exitg4 = false;
                while ((!exitg4) && (idx <= nA - 1)) {
                  if (r[idx] >= 1.0E-12) {
                    DualFeasible = false;
                    exitg4 = true;
                  } else {
                    idx++;
                  }
                }
              }
              if ((nA != 0) && (!DualFeasible)) {
                for (i = 0; i < nA; i++) {
                  t = r[i];
                  if (t > 1.0E-12) {
                    cVal = lambda[iC[i] - 1] / t;
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = i + 1;
                    }
                  }
                }
                if (kDrop > 0) {
                  cMin = rMin;
                  ColdReset = false;
                }
              }
              cVal = 0.0;
              for (k = 0; k < 7; k++) {
                cVal += z[k] * Ac[tmp + 72 * k];
              }
              if (cVal <= 0.0) {
                cVal = 0.0;
                DualFeasible = true;
              } else {
                t = 0.0;
                for (b_i = 0; b_i < 7; b_i++) {
                  t += Ac[tmp + 72 * b_i] * x[b_i];
                }
                cVal = (b[tmp] - t) / cVal;
                DualFeasible = false;
              }
              if (ColdReset && DualFeasible) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (DualFeasible) {
                  t = cMin;
                } else if (ColdReset) {
                  t = cVal;
                } else if (cMin < cVal) {
                  t = cMin;
                } else {
                  t = cVal;
                }
                for (i = 0; i < nA; i++) {
                  lambda[iC[i] - 1] -= t * r[i];
                  if (lambda[iC[i] - 1] < 0.0) {
                    lambda[iC[i] - 1] = 0.0;
                  }
                }
                lambda[tmp] += t;
                frexp(1.0, &exponent);
                if (std::abs(t - cMin) < 2.2204460492503131E-16) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }
                if (!DualFeasible) {
                  for (b_i = 0; b_i < 7; b_i++) {
                    x[b_i] += t * z[b_i];
                  }
                  frexp(1.0, &b_exponent);
                  if (std::abs(t - cVal) < 2.2204460492503131E-16) {
                    if (nA == 7) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      if (nA > 2147483646) {
                        nA = MAX_int32_T;
                      } else {
                        nA++;
                      }
                      iC[nA - 1] = tmp + 1;
                      i = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (i + 1 > 1)) {
                        b_i = iC[i - 1];
                        if (iC[i] > b_i) {
                          exitg4 = true;
                        } else {
                          idx = iC[i];
                          iC[i] = b_i;
                          iC[i - 1] = idx;
                          i--;
                        }
                      }
                      iA[tmp] = true;
                      tmp = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cVal = b_norm(x);
            if (std::abs(cVal - Xnorm0) > 0.001) {
              Xnorm0 = cVal;
              for (k = 0; k < 72; k++) {
                cTol[k] = std::fmax(std::abs(b[k]), 1.0);
              }
              cTolComputed = false;
            }
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

} // namespace coder

// End of code generation (qpkwik.cpp)
