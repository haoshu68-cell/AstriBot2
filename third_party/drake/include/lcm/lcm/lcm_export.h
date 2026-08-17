#ifndef LCM_EXPORT_H
#define LCM_EXPORT_H

#ifdef LCM_STATIC
#  define LCM_EXPORT
#  define LCM_NO_EXPORT
#else
#  define LCM_EXPORT __attribute__((visibility("default")))
#  define LCM_NO_EXPORT __attribute__((visibility("hidden")))
#endif

#ifndef LCM_DEPRECATED
#  define LCM_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef LCM_DEPRECATED_EXPORT
#  define LCM_DEPRECATED_EXPORT LCM_EXPORT LCM_DEPRECATED
#endif

#ifndef LCM_DEPRECATED_NO_EXPORT
#  define LCM_DEPRECATED_NO_EXPORT LCM_NO_EXPORT LCM_DEPRECATED
#endif

#endif
