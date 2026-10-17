#ifndef CORE_LINKAGE_H
#define CORE_LINKAGE_H

// Explicit ABI declarations shared by C callers and C++ implementations.
#ifdef __cplusplus
    #define LF_EXTERN_C extern "C"
#else
    #define LF_EXTERN_C extern
#endif

#endif // CORE_LINKAGE_H
