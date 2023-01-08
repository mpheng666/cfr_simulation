//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// xgerc.cpp
//
// Code generation for function 'xgerc'
//

// Include files
#include "xgerc.h"

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgerc(int m, int n, double alpha1, int ix0, const double y[7],
           double A[49], int ia0)
{
  if (!(alpha1 == 0.0)) {
    int jA;
    jA = ia0;
    for (int j{0}; j < n; j++) {
      double temp;
      temp = y[j];
      if (temp != 0.0) {
        int i;
        temp *= alpha1;
        i = m + jA;
        for (int ijA{jA}; ijA < i; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += 7;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgerc.cpp)
