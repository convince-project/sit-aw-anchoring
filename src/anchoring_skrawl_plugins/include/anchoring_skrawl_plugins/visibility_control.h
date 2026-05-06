#ifndef ANCHORING_SKRAWL_PLUGINS__VISIBILITY_CONTROL_H_
#define ANCHORING_SKRAWL_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ANCHORING_SKRAWL_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define ANCHORING_SKRAWL_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define ANCHORING_SKRAWL_PLUGINS_EXPORT __declspec(dllexport)
    #define ANCHORING_SKRAWL_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef ANCHORING_SKRAWL_PLUGINS_BUILDING_LIBRARY
    #define ANCHORING_SKRAWL_PLUGINS_PUBLIC ANCHORING_SKRAWL_PLUGINS_EXPORT
  #else
    #define ANCHORING_SKRAWL_PLUGINS_PUBLIC ANCHORING_SKRAWL_PLUGINS_IMPORT
  #endif
  #define ANCHORING_SKRAWL_PLUGINS_PUBLIC_TYPE ANCHORING_SKRAWL_PLUGINS_PUBLIC
  #define ANCHORING_SKRAWL_PLUGINS_LOCAL
#else
  #define ANCHORING_SKRAWL_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define ANCHORING_SKRAWL_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define ANCHORING_SKRAWL_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define ANCHORING_SKRAWL_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ANCHORING_SKRAWL_PLUGINS_PUBLIC
    #define ANCHORING_SKRAWL_PLUGINS_LOCAL
  #endif
  #define ANCHORING_SKRAWL_PLUGINS_PUBLIC_TYPE
#endif

#endif  // ANCHORING_SKRAWL_PLUGINS__VISIBILITY_CONTROL_H_
