
#ifndef CURIO_ONE__VISIBILITY_CONTROL_H_
#define CURIO_ONE__VISIBILITY_CONTROL_H_



#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CURIO_ONE_EXPORT __attribute__((dllexport))
#define CURIO_ONE_IMPORT __attribute__((dllimport))
#else
#define CURIO_ONE_EXPORT __declspec(dllexport)
#define CURIO_ONE_IMPORT __declspec(dllimport)
#endif
#ifdef CURIO_ONE_BUILDING_DLL
#define CURIO_ONE_PUBLIC CURIO_ONE_EXPORT
#else
#define CURIO_ONE_PUBLIC CURIO_ONE_IMPORT
#endif
#define CURIO_ONE_PUBLIC_TYPE CURIO_ONE_PUBLIC
#define CURIO_ONE_LOCAL
#else
#define CURIO_ONE_EXPORT __attribute__((visibility("default")))
#define CURIO_ONE_IMPORT
#if __GNUC__ >= 4
#define CURIO_ONE_PUBLIC __attribute__((visibility("default")))
#define CURIO_ONE_LOCAL __attribute__((visibility("hidden")))
#else
#define CURIO_ONE_PUBLIC
#define CURIO_ONE_LOCAL
#endif
#define CURIO_ONE_PUBLIC_TYPE
#endif

#endif  // CURIO_ONE__VISIBILITY_CONTROL_H_
