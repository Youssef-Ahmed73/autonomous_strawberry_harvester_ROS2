#ifndef VISION__VISIBILITY_CONTROL_H_
#define VISION__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VISION_EXPORT __attribute__ ((dllexport))
    #define VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define VISION_EXPORT __declspec(dllexport)
    #define VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef VISION_BUILDING_LIBRARY
    #define VISION_PUBLIC VISION_EXPORT
  #else
    #define VISION_PUBLIC VISION_IMPORT
  #endif
  #define VISION_PUBLIC_TYPE VISION_PUBLIC
  #define VISION_LOCAL
#else
  #define VISION_EXPORT __attribute__ ((visibility("default")))
  #define VISION_IMPORT
  #if __GNUC__ >= 4
    #define VISION_PUBLIC __attribute__ ((visibility("default")))
    #define VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VISION_PUBLIC
    #define VISION_LOCAL
  #endif
  #define VISION_PUBLIC_TYPE
#endif

#endif  // VISION__VISIBILITY_CONTROL_H_