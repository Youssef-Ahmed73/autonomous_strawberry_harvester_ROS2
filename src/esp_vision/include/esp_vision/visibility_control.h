#ifndef ESP_VISION__VISIBILITY_CONTROL_H_
#define ESP_VISION__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ESP_VISION_EXPORT __attribute__ ((dllexport))
    #define ESP_VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define ESP_VISION_EXPORT __declspec(dllexport)
    #define ESP_VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef ESP_VISION_BUILDING_LIBRARY
    #define ESP_VISION_PUBLIC ESP_VISION_EXPORT
  #else
    #define ESP_VISION_PUBLIC ESP_VISION_IMPORT
  #endif
  #define ESP_VISION_PUBLIC_TYPE ESP_VISION_PUBLIC
  #define ESP_VISION_LOCAL
#else
  #define ESP_VISION_EXPORT __attribute__ ((visibility("default")))
  #define ESP_VISION_IMPORT
  #if __GNUC__ >= 4
    #define ESP_VISION_PUBLIC __attribute__ ((visibility("default")))
    #define ESP_VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ESP_VISION_PUBLIC
    #define ESP_VISION_LOCAL
  #endif
  #define ESP_VISION_PUBLIC_TYPE
#endif

#endif  // ESP_VISION__VISIBILITY_CONTROL_H_