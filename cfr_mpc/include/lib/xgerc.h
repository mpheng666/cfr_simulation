//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// xgerc.h
//
// Code generation for function 'xgerc'
//

#ifndef XGERC_H
#define XGERC_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgerc(int m, int n, double alpha1, int ix0, const double y[7],
           double A[49], int ia0);

}
} // namespace internal
} // namespace coder

#endif
// End of code generation (xgerc.h)
