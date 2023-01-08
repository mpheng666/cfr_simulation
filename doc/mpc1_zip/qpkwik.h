//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// qpkwik.h
//
// Code generation for function 'qpkwik'
//

#ifndef QPKWIK_H
#define QPKWIK_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void qpkwik(const double Linv[49], const double Hinv[49], const double f[7],
            const double Ac[504], const double b[72], boolean_T iA[72],
            double x[7], double lambda[72], int *status);

}

#endif
// End of code generation (qpkwik.h)
