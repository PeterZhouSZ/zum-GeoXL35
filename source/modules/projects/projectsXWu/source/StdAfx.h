#ifndef PCH_PROJECTSXWU_H
#define PCH_PROJECTSXWU_H


// note: the header file is not located in the subdirectory x4basics because the msvc compiler does not 
//       handle directory prefixes on precompiled header files

/// precompiled header file that includes common headers from the standard C++ library, Windows, and QT Core,Gui
/// precompiled headers can be shared among several projects

////////////////////////////////////////////////
//                                            //
//                  Global                    //
//                                            //
////////////////////////////////////////////////

#include <X4GlobalDefinitions.h>

////////////////////////////////////////////////
//                                            //
//                  Windows                   //
//                                            //
////////////////////////////////////////////////

#if defined(WIN32) || defined(WIN64)

    #ifndef WINVER                          // Specifies that the minimum required platform is Windows Vista.
    #define WINVER 0x0600           // Change this to the appropriate value to target other versions of Windows.
    #endif

    #ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
    #define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
    #endif

    #ifndef _WIN32_WINDOWS          // Specifies that the minimum required platform is Windows 98.
    #define _WIN32_WINDOWS 0x0410 // Change this to the appropriate value to target Windows Me or later.
    #endif

    #ifndef _WIN32_IE                       // Specifies that the minimum required platform is Internet Explorer 7.0.
    #define _WIN32_IE 0x0700        // Change this to the appropriate value to target other versions of IE.
    #endif

    #ifndef NOMINMAX
    #define NOMINMAX
    #endif

    #define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
    // Windows Header Files:
    #include <windows.h>
    #include <mmsystem.h>

#endif


////////////////////////////////////////////////
//                                            //
//             Annoying Warnings              //
//                                            //
////////////////////////////////////////////////

#pragma warning (disable : 4819) /*  The file contains a character that cannot be represented in the current code page */
#pragma warning (disable : 4305) /*  truncation from 'double' to 'float' */
#pragma warning (disable : 4267) /*  conversion from 'size_t' to 'unsigned int', possible loss of data' */
#pragma warning (disable : 4996) // Function call with parameters that may be unsafe - this call relies on the caller to check that the passed values are correct.

////////////////////////////////////////////////
//                                            //
//                 C++ / STD                  //
//                                            //
////////////////////////////////////////////////

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

// standard template library
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <list>
#include <deque>
#include <algorithm>
#include <map>
#include <set>
#include <complex>

#include <math_include.h>
#include <values_vs.h>

////////////////////////////////////////////////
//                                            //
//                 OpenGL                     //
//                                            //
////////////////////////////////////////////////
#include <gl.h>

////////////////////////////////////////////////
//                                            //
//                    QT                      //
//                                            //
////////////////////////////////////////////////
#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <QtOpenGL/QGLWidget>

////////////////////////////////////////////////
//                                            //
//                    X4                      //
//                                            //
////////////////////////////////////////////////

#include "x4core.all.h"
#include "Tools.h"
#include "X4Types.h"
#include "MetaClass.h"
#include "Persistent.h"
#include "LinearAlgebra.h"
#include "X4Exception.h"

#include "DebugOutput.h"
#include "DebugRenderer.h"
#include "Timer.h"
#include "ProgressWindow.h"

#endif

