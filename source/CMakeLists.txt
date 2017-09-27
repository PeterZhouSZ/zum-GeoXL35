if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)
SET (CMAKE_CONFIGURATION_TYPES "Debug;Release;RelWithDebInfo;" CACHE STRING "possible configurations" FORCE)
SET (CMAKE_BUILD_TYPE Debug CACHE STRING "Default build type" FORCE)
LIST (APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/")

################################################################################
## project info
################################################################################
PROJECT (GeoX)
STRING (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
SET (X_VERSION_MAJOR 3)
SET (X_VERSION_MINOR 6)
SET (X_VERSION ${X_VERSION_MAJOR}.${X_VERSION_MINOR})
SET (SystemVersionInfo "${PROJECT_NAME_UPPER}_${X_VERSION_MAJOR}_${X_VERSION_MINOR}")

################################################################################
## output directiory
################################################################################
STRING(REGEX REPLACE " " "-" SYS_CFG ${CMAKE_GENERATOR})
SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/lib/${SYS_CFG}")
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/lib/${SYS_CFG}")
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/bin/${SYS_CFG}")
SET (EXECUTABLE_OUTPUT_PATH "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_CFG_INTDIR}")
SET (EXECUTABLE_OUTPUT_PATH_DEBUG "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug")
SET (EXECUTABLE_OUTPUT_PATH_RELEASE "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release")
SET (EXECUTABLE_OUTPUT_PATH_RelWithDebInfo "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/RelWithDebInfo")
SET (X_RES_DIR "${CMAKE_BINARY_DIR}/stage/resources")
SET (X_CFG_DIR "${CMAKE_BINARY_DIR}/stage/config")
SET (X_DOC_DIR "${CMAKE_BINARY_DIR}/stage/docs")
SET (X_LOG_DIR "${CMAKE_BINARY_DIR}/stage/log")

################################################################################
## project directory management
################################################################################
SET_PROPERTY (GLOBAL PROPERTY USE_FOLDERS ON)
INCLUDE (${CMAKE_SOURCE_DIR}/cmake/macros.cmake)

################################################################################
## global definitions
################################################################################
ADD_DEFINITIONS (-D"${SystemVersionInfo}")
IF (WIN32)
	SET (CMAKE_CXX_FLAGS "/D_WINDOWS /W3 /GR /EHsc" CACHE STRING "removed WIN32" FORCE)
	SET (CMAKE_C_FLAGS "/D_WINDOWS /W3" CACHE STRING "removed WIN32" FORCE)
	ADD_DEFINITIONS (-DUNICODE -D_UNICODE)
	#ADD_DEFINITIONS(-DNOMINMAX)
	#ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)

	IF (CMAKE_CL_64)
		SET (PlatformName "x64")
		ADD_DEFINITIONS (-DWIN64 -D_WIN64)
	ELSE ()
		SET (PlatformName "win32")
		ADD_DEFINITIONS (-DWIN32 -D_WIN32)
	ENDIF ()
	
	IF (MSVC)
		ADD_DEFINITIONS (-DX_PLATFORM_MSVC -DX4_PLATFORM_MSVC)
	ENDIF ()
	
ELSEIF (UNIX AND NOT APPLE)
	ADD_DEFINITIONS (-DX_PLATFORM_LINUX_GCC)
	ADD_DEFINITIONS (-DUNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	
	# lfs on glibc, all compilers should use
	add_definitions(-D_LARGEFILE_SOURCE -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE)
	
	# GNU Compiler
	if(CMAKE_COMPILER_IS_GNUCC)
		set(PLATFORM_CFLAGS "-pipe -fPIC -funsigned-char -fno-strict-aliasing")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
		SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

		# use ld.gold linker if available, could make optional
		execute_process(
		        COMMAND ${CMAKE_C_COMPILER} -fuse-ld=gold -Wl,--version
		        ERROR_QUIET OUTPUT_VARIABLE LD_VERSION)
		if("${LD_VERSION}" MATCHES "GNU gold")
			set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fuse-ld=gold")
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
		else()
			message(STATUS "GNU gold linker isn't available, using the default system linker.")
		endif()
		unset(LD_VERSION)
	endif()
ENDIF ()
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS $<$<CONFIG:Debug>:X_DEBUG>)

################################################################################
## 3rd party library
################################################################################
OPTION (USE_OPENCV "use OpenCV" OFF)
IF (WIN32)
	IF (MSVC_VERSION EQUAL 1900) # VS2015
		SET (QT_DIR "C:/Qt" CACHE STRING "QT" FORCE)
		SET (BOOST_INCLUDEDIR "C:/boost/include/boost-1_59" CACHE STRING "Boost inlude directory" FORCE)
		SET (BOOST_LIBRARYDIR "C:/boost/lib" CACHE STRING "Boost library directiory" FORCE)
		SET (USE_OPENCV ON)
		SET (OpenCV_DIR "C:/opencv/build" CACHE STRING "OpenCV" FORCE)
		SET (OpenCV_BIN "C:/opencv/build/x64/vc14/bin" CACHE STRING "OpenCV" FORCE)
	ELSE (MSVC_VERSION EQUAL 1900) # VS2013
		SET (QT_DIR "C:/Qt13/5.5/msvc2013_64" CACHE STRING "QT" FORCE)
		SET (BOOST_INCLUDEDIR "C:/local/boost_1_59_0" CACHE STRING "Boost inlude directory" FORCE)
		SET (BOOST_LIBRARYDIR "C:/local/boost_1_59_0/lib64-msvc-12.0" CACHE STRING "Boost library directiory" FORCE)
		SET (USE_OPENCV ON)
		SET (OpenCV_DIR "C:/opencv/build" CACHE STRING "OpenCV" FORCE)
		SET (OpenCV_BIN "C:/opencv/build/x64/vc12/bin" CACHE STRING "OpenCV" FORCE)
	ENDIF ()
ENDIF ()
OPTION (USE_CUDA "use cuda" ON)
SET (QT_VERSION 5 CACHE STRING "qt version, currently 4 or 5" FORCE)
INCLUDE (${CMAKE_SOURCE_DIR}/cmake/find3rd.cmake)

################################################################################
## Add the projects
################################################################################
SET (module_dir "${CMAKE_SOURCE_DIR}/modules")
SET (pch_basename "precompiledHeadersBasics")
SET (pch_std_name "StdAfx")
SET (pch_path "${module_dir}/basics/x4basics/source")
SET (LIST_INCS_CMAKE "${CMAKE_BINARY_DIR}/common/list_incs_dirs")
IF (WIN32)
	SET (ModuleParserProgram "x4ModuleParser.exe")
ELSEIF (UNIX AND NOT APPLE)
	SET (ModuleParserProgram "x4ModuleParser")
ENDIF ()
SET (ModuleParserProject "x4ModuleParser")
INCLUDE (${CMAKE_SOURCE_DIR}/cmake/listFile.cmake)
LIST_INCS (
    ${LIST_INCS_CMAKE} ${module_dir}
    )

INCLUDE (${CMAKE_SOURCE_DIR}/cmake/projBody.cmake)
collect_subprojects (${module_dir})
