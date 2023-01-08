//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include files
#include "main.h"
#include "mpcmoveCodeGeneration.h"
#include "mpcmoveCodeGeneration_terminate.h"
#include "mpcmoveCodeGeneration_types.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);

static void argInit_6x6_real_T(double result[36]);

static void argInit_72x1_boolean_T(boolean_T result[72]);

static boolean_T argInit_boolean_T();

static double argInit_real_T();

static void argInit_struct4_T(struct4_T *result);

static struct5_T argInit_struct5_T();

static struct6_T argInit_struct6_T();

static void main_mpcmoveCodeGeneration();

// Function Definitions
static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_6x6_real_T(double result[36])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 6; idx0++) {
    for (int idx1{0}; idx1 < 6; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 6 * idx1] = argInit_real_T();
    }
  }
}

static void argInit_72x1_boolean_T(boolean_T result[72])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 72; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_boolean_T();
  }
}

static boolean_T argInit_boolean_T()
{
  return false;
}

static double argInit_real_T()
{
  return 0.0;
}

static void argInit_struct4_T(struct4_T *result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result->Plant);
  argInit_6x6_real_T(result->Covariance);
  argInit_72x1_boolean_T(result->iA);
  result->Disturbance[0] = result->Plant[0];
  result->LastMove[0] = result->Plant[0];
  result->Disturbance[1] = result->Plant[1];
  result->LastMove[1] = result->Plant[1];
  result->Disturbance[2] = result->Plant[2];
  result->LastMove[2] = result->Plant[2];
}

static struct5_T argInit_struct5_T()
{
  struct5_T result;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.signals = argInit_struct6_T();
  return result;
}

static struct6_T argInit_struct6_T()
{
  struct6_T result;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result.ym);
  argInit_3x1_real_T(result.ref);
  return result;
}

static void main_mpcmoveCodeGeneration()
{
  struct10_T Info;
  struct4_T statedata;
  struct5_T r;
  double u[3];
  // Initialize function 'mpcmoveCodeGeneration' input arguments.
  // Initialize function input argument 'statedata'.
  // Initialize function input argument 'onlinedata'.
  // Call the entry-point 'mpcmoveCodeGeneration'.
  argInit_struct4_T(&statedata);
  r = argInit_struct5_T();
  coder::mpcmoveCodeGeneration(&statedata, &r, u, &Info);
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_mpcmoveCodeGeneration();
  // Terminate the application.
  // You do not need to do this more than one time.
  mpcmoveCodeGeneration_terminate();
  return 0;
}

// End of code generation (main.cpp)
