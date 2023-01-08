//
// Community and Technical College License -- for use in teaching and
// meeting course requirements at community and technical colleges only.
// Not for government, commercial, university, or other organizational
// use.
//
// mpcmoveCodeGeneration_spec.h
//
// Code generation for function 'mpcmoveCodeGeneration'
//

#ifndef MPCMOVECODEGENERATION_SPEC_H
#define MPCMOVECODEGENERATION_SPEC_H

// Include files
#ifdef MPCMOVECODEGENERATION_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define MPCMOVECODEGENERATION_DLL_EXPORT __declspec(dllimport)
#else
#define MPCMOVECODEGENERATION_DLL_EXPORT __attribute__((visibility("default")))
#endif
#elif defined(BUILDING_MPCMOVECODEGENERATION)
#if defined(_MSC_VER) || defined(__LCC__)
#define MPCMOVECODEGENERATION_DLL_EXPORT __declspec(dllexport)
#else
#define MPCMOVECODEGENERATION_DLL_EXPORT __attribute__((visibility("default")))
#endif
#else
#define MPCMOVECODEGENERATION_DLL_EXPORT
#endif

#endif
// End of code generation (mpcmoveCodeGeneration_spec.h)
