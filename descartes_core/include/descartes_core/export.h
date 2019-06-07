#ifndef DESCARTES_CORE_EXPORT_H_
#define DESCARTES_CORE_EXPORT_H_

#include <ros/macros.h>

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef descartes_core_EXPORTS // we are building a shared lib/dll
    #define DESCARTES_CORE_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define DESCARTES_CORE_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define DESCARTES_CORE_DECL
#endif

#endif