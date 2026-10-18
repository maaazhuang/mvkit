#pragma once

#if defined(_WIN32)
  #if defined(MVKIT_CPP_BUILD_SHARED)
    #define MVKIT_CPP_API __declspec(dllexport)
  #elif defined(MVKIT_CPP_USE_SHARED)
    #define MVKIT_CPP_API __declspec(dllimport)
  #else
    #define MVKIT_CPP_API
  #endif
#elif defined(__GNUC__) || defined(__clang__)
  #define MVKIT_CPP_API __attribute__((visibility("default")))
#else
  #define MVKIT_CPP_API
#endif
