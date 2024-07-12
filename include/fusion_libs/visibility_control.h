#ifndef FUSION___VISIBILITY_CONTROL_H_
#define FUSION___VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define FUSION__EXPORT __attribute__ ((dllexport))
    #define FUSION__IMPORT __attribute__ ((dllimport))
  #else
    #define FUSION__EXPORT __declspec(dllexport)
    #define FUSION__IMPORT __declspec(dllimport)
  #endif
  #ifdef FUSION__BUILDING_LIBRARY
    #define FUSION__PUBLIC FUSION__EXPORT
  #else
    #define FUSION__PUBLIC FUSION__IMPORT
  #endif
  #define FUSION__PUBLIC_TYPE FUSION__PUBLIC
  #define FUSION__LOCAL
#else
  #define FUSION__EXPORT __attribute__ ((visibility("default")))
  #define FUSION__IMPORT
  #if __GNUC__ >= 4
    #define FUSION__PUBLIC __attribute__ ((visibility("default")))
    #define FUSION__LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define FUSION__PUBLIC
    #define FUSION__LOCAL
  #endif
  #define FUSION__PUBLIC_TYPE
#endif

#endif  // FUSION___VISIBILITY_CONTROL_H_
