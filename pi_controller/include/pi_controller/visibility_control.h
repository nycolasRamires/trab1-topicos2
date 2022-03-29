#ifndef PI_CONTROLLER__VISIBILITY_CONTROL_H_
#define PI_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PI_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define PI_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define PI_CONTROLLER_EXPORT __declspec(dllexport)
    #define PI_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PI_CONTROLLER_BUILDING_LIBRARY
    #define PI_CONTROLLER_PUBLIC PI_CONTROLLER_EXPORT
  #else
    #define PI_CONTROLLER_PUBLIC PI_CONTROLLER_IMPORT
  #endif
  #define PI_CONTROLLER_PUBLIC_TYPE PI_CONTROLLER_PUBLIC
  #define PI_CONTROLLER_LOCAL
#else
  #define PI_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define PI_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define PI_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define PI_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PI_CONTROLLER_PUBLIC
    #define PI_CONTROLLER_LOCAL
  #endif
  #define PI_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // PI_CONTROLLER__VISIBILITY_CONTROL_H_
