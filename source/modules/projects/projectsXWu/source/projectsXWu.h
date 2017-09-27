#ifndef PROJECTSXWU_MODULE_H
#define PROJECTSXWU_MODULE_H

#include <X4GlobalDefinitions.h>


#if defined (X_PLATFORM_LINUX_GCC)
    #define PROJECTSXWU_API __attribute__ ((__visibility__("default")))
#elif defined (PLATFORM_WINDOWS)
    #ifdef PROJECTSXWU_EXPORTS
    #define PROJECTSXWU_API __declspec(dllexport)
    #else
    #define PROJECTSXWU_API __declspec(dllimport)
    #endif
#endif

#endif
