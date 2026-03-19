#ifndef ANCHORING_CUBESWORLD_PLUGIN__VISIBILITY_CONTROL_H_
#define ANCHORING_CUBESWORLD_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ANCHORING_CUBESWORLD_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define ANCHORING_CUBESWORLD_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define ANCHORING_CUBESWORLD_PLUGIN_EXPORT __declspec(dllexport)
    #define ANCHORING_CUBESWORLD_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef ANCHORING_CUBESWORLD_PLUGIN_BUILDING_LIBRARY
    #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC ANCHORING_CUBESWORLD_PLUGIN_EXPORT
  #else
    #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC ANCHORING_CUBESWORLD_PLUGIN_IMPORT
  #endif
  #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC_TYPE ANCHORING_CUBESWORLD_PLUGIN_PUBLIC
  #define ANCHORING_CUBESWORLD_PLUGIN_LOCAL
#else
  #define ANCHORING_CUBESWORLD_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define ANCHORING_CUBESWORLD_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define ANCHORING_CUBESWORLD_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC
    #define ANCHORING_CUBESWORLD_PLUGIN_LOCAL
  #endif
  #define ANCHORING_CUBESWORLD_PLUGIN_PUBLIC_TYPE
#endif

#endif  // ANCHORING_CUBESWORLD_PLUGIN__VISIBILITY_CONTROL_H_
