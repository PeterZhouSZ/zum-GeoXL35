#ifndef PCH_PROJECTSEG2014_H
#define PCH_PROJECTSEG2014_H


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

	#define NOMINMAX
	#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
	// Windows Header Files:
	#include <windows.h>

   #include <mmsystem.h>

#endif


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
#include <iostream>
#include <string>
#include <list>
#include <deque>
#include <algorithm>
#include <vector>
#include <map>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>
#include <sstream>

////////////////////////////////////////////////
//                                            //
//                    QT                      //
//                                            //
////////////////////////////////////////////////

#include <QtCore/QtCore>


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

#include "PropertyEditor.h"
#include "Scene.h"
#include "BoundingBox.h"
#include "BoundingBox.inline.h"
#include "LinearAlgebra.h"
#include "LinearAlgebra.inline.h"
#include "PointCloud.h"
#include "PointCloudIterators.h"
#include "NumericalClassProperty.h"
#include "FixedArrayClassProperty.h"
#include "StringClassProperty.h"
#include "PointSet.h"
#include "PointSet.inline.h"
#include "UnstructuredInCorePointCloud.h"
#include "DebugOutput.h"
#include "VertexDescriptor.h"
#include "Random.h"
#include "PolarCoordinates.h"
#include "PointCloudIterators.h"
#include "BooleanClassProperty.h"
#include "CommandHistory.h"
#include "PCInteractionTool.h"
#include "CameraController.h"
#include "ExaminerCameraController.h"
#include "ObjectClassProperty.h"
#include "Scene.h"
#include "SceneGraphState.h"
#include "DebugOutput.h"
#include "PointCloud.h"
#include "PointCloudIterators.h"
#include "NumericalClassProperty.h"
#include "PointSet.h"
#include "SceneCommandObjects.h"
#include "CommandHistory.h"
#include "SelectionIteratorAdaptor.h"
#include "ObjectViewsTable.h"
#include "SGLegacyPCSceneAdaptor.h"
#include "MathTools.h"
#include "BooleanClassProperty.h"
#include "NumericalClassProperty.h"
#include "StringClassProperty.h"
#include "FixedArrayClassProperty.h"
#include "LinearAlgebra.inline.h"
#include "DebugRenderer.h"
#include "SGObjectNode.h"
#include "PCCAutoAlign.h"
#include "PointCloudTools.h"
#include "PCCTriangleMeshSampler.h"
#include "VertexArray.h"
#include "ANN.h"
#include "AnnSearch.h"
#include "PointSetKNNQuery.h"
#include "SGObjectNode.h"
#include "SGListNode.h"
#include "SceneGraphTools.h"
#include "RegularVolume.h"
#include "RegularInCoreVolume.h"
#include "RegularVolume.inline.h"
#include "PointSetANNQuery.h"
#include "Timer.h"

#endif