SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules)
#MESSAGE("CMAKE_CURRENT_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}")

################################################################################
## Boost
################################################################################
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
#set(Boost_DEBUG TRUE)
# Disable the config mode of find_package(Boost)
#set(Boost_NO_BOOST_CMAKE ON)
# Required boost modules
find_package(Boost REQUIRED
    COMPONENTS system filesystem thread date_time iostreams
    )
if (NOT Boost_FOUND)
	# SET (BOOST_INCLUDEDIR "${CMAKE_SOURCE_DIR}/thirdParty/Boost/inc" CACHE STRING "Boost" FORCE)
	# SET (BOOST_LIBRARYDIR "${CMAKE_SOURCE_DIR}/thirdParty/Boost/lib" CACHE STRING "Boost" FORCE)
	# find_package(Boost #REQUIRED
		# COMPONENTS system filesystem thread date_time iostreams
		# )
	if (NOT Boost_FOUND)
		MESSAGE(FATAL_ERROR "Boost was not found.")
	endif ()
else ()
    # Obtain diagnostic information about Boost's automatic linking outputted 
    # during compilation time.
    add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    SET(Boost_LIB_Core
        "boost_date_time"
        "boost_system"
        "boost_thread"
        "boost_chrono"
        )
	IF (WIN32)
		SET (Boost_LIB_Core_Debug )
		FOREACH (dll_file ${Boost_DLL_Core})
			FILE(GLOB inFiles RELATIVE "${Boost_LIBRARY_DIR_DEBUG}"
				"${Boost_LIBRARY_DIR_DEBUG}/${dll_file}-vc*mt-gd*.dll")
			SET (Boost_LIB_Core_Debug ${Boost_LIB_Core_Debug} ${inFiles})
		ENDFOREACH ()
		SET (Boost_LIB_Core_Release )
		FOREACH (dll_file ${Boost_DLL_Core})
			FILE(GLOB inFiles RELATIVE "${Boost_LIBRARY_DIR_RELEASE}"
				"${Boost_LIBRARY_DIR_RELEASE}/${dll_file}-vc*mt-[1-9]*.dll")
			SET (Boost_LIB_Core_Release ${Boost_LIB_Core_Release} ${inFiles})
		ENDFOREACH ()
	ELSEIF (UNIX AND NOT APPLE)
		SET (Boost_LIB_Core_Debug )
		FOREACH (dll_file ${Boost_DLL_Core})
			FILE(GLOB inFiles RELATIVE "${Boost_LIBRARY_DIR_DEBUG}"
				"${Boost_LIBRARY_DIR_DEBUG}/lib${dll_file}.so")
			SET (Boost_LIB_Core_Debug ${Boost_LIB_Core_Debug} ${inFiles})
		ENDFOREACH ()
		SET (Boost_LIB_Core_Release )
		FOREACH (dll_file ${Boost_DLL_Core})
			FILE(GLOB inFiles RELATIVE "${Boost_LIBRARY_DIR_RELEASE}"
				"${Boost_LIBRARY_DIR_RELEASE}/lib${dll_file}.so")
			SET (Boost_LIB_Core_Release ${Boost_LIB_Core_Release} ${inFiles})
		ENDFOREACH ()
	ENDIF ()
endif ()


################################################################################
## CLAPACK
################################################################################
SET (CLAPACK_INCLUDE "${CMAKE_SOURCE_DIR}/modules/thirdParty/CLAPACK/include")
SET (CLAPACK_LIBRARY
    "${CMAKE_SOURCE_DIR}/modules/thirdParty/CLAPACK/lib/${PlatformName}/${CMAKE_CFG_INTDIR}/blas.lib"
    "${CMAKE_SOURCE_DIR}/modules/thirdParty/CLAPACK/lib/${PlatformName}/${CMAKE_CFG_INTDIR}/lapack.lib"
    "${CMAKE_SOURCE_DIR}/modules/thirdParty/CLAPACK/lib/${PlatformName}/${CMAKE_CFG_INTDIR}/libf2c.lib"
    )
	
	
################################################################################
## Nvidia CG
################################################################################
FIND_PACKAGE(Cg REQUIRED)
IF (NOT CG_FOUND)
    MESSAGE(FATAL_ERROR "Cg was not found.")
ELSE ()
	MESSAGE(STATUS "using Nvidia CG")
ENDIF ()


################################################################################
## CUDA
################################################################################
if(USE_CUDA)
    #include(${CMAKE_SOURCE_DIR}/cmake/find_cuda.cmake)
    find_package(CUDA)
    if(CUDA_FOUND)
        ADD_DEFINITIONS(-DUSE_CUDA)
		#MESSAGE(STATUS "using CUDA")
    else(CUDA_FOUND)
        SET(USE_CUDA OFF)
		#MESSAGE(STATUS "CUDA not found")
    endif(CUDA_FOUND)
endif(USE_CUDA)


################################################################################
## Eigen
################################################################################
#find_package(Eigen3 #REQUIRED)
SET (Eigen_INCLUDE "${CMAKE_SOURCE_DIR}/thirdParty/Eigen")
IF (EXISTS ${Eigen_INCLUDE}/Eigen)
	MESSAGE(STATUS "using Eigen")
ELSE ()
	MESSAGE(FATAL_ERROR "Eigen not found")
ENDIF ()


################################################################################
## Fastest Fourier Transform in the West (short name: FFTW)
################################################################################
SET (FFTW_BIN "${CMAKE_SOURCE_DIR}/modules/thirdParty/libfftw3/bin/${PlatformName}")
SET (FFTW_LIBRARY
    "${CMAKE_SOURCE_DIR}/modules/thirdParty/libfftw3/lib/${PlatformName}/libfftw.lib"
    "${CMAKE_SOURCE_DIR}/modules/thirdParty/libfftw3/lib/${PlatformName}/libfftwf.lib"
    )
SET (FFTW_DLL
    "${FFTW_BIN}/libfftw_64.dll"
    "${FFTW_BIN}/libfftwf_64.dll"
    )
	
################################################################################
## OpenCV
################################################################################
IF (USE_OPENCV)
	SET(BUILD_SHARED_LIBS ON)
	FIND_PACKAGE(OpenCV REQUIRED)
	IF (NOT OpenCV_FOUND)
		MESSAGE(FATAL_ERROR "OpenCV was not found.")
	ENDIF(NOT OpenCV_FOUND)
	SET(OpenCV_DLL
		"opencv_core"
		"opencv_highgui"
		"opencv_world"
		)
	IF (WIN32)
		SET (OpenCV_LIB_Core_Debug )
		FOREACH (dll_file ${OpenCV_DLL})
			FILE(GLOB inFiles RELATIVE "${OpenCV_BIN}"
				"${OpenCV_BIN}/${dll_file}[1-9]*d.dll")
			SET (OpenCV_LIB_Core_Debug ${OpenCV_LIB_Core_Debug} ${inFiles})
		ENDFOREACH ()
		SET (OpenCV_LIB_Core_Release )
		FOREACH (dll_file ${OpenCV_DLL})
			FILE(GLOB inFiles RELATIVE "${OpenCV_BIN}"
				"${OpenCV_BIN}/${dll_file}[1-9]*[^d].dll")
			SET (OpenCV_LIB_Core_Release ${OpenCV_LIB_Core_Release} ${inFiles})
		ENDFOREACH ()
		
		#message(${OpenCV_LIB_Core_Debug})
		#message(${OpenCV_LIB_Core_Release})
	ENDIF ()
ENDIF ()

################################################################################
## OpenGL
################################################################################
FIND_PACKAGE(OpenGL REQUIRED)

################################################################################
## OpenMP
################################################################################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################################################
## Qt
## QT5
################################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_AUTOMOC ON)
IF (WIN32)
	SET(QT_BIN "${QT_DIR}/bin")
	set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${QT_DIR}/lib/cmake")
ENDIF ()
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Xml REQUIRED)
set(QT_LIBRARIES "Qt5::Widgets;Qt5::Core;Qt5::Gui;Qt5::OpenGL;Qt5::Xml")

SET(QT_DLL_DEBUG
	"Qt5Widgetsd.dll"
	"Qt5Cored.dll"
	"Qt5Guid.dll"
	"Qt5OpenGLd.dll"
	"Qt5Xmld.dll"
	)
SET(QT_DLL_RELEASE
	"Qt5Widgets.dll"
	"Qt5Core.dll"
	"Qt5Gui.dll"
	"Qt5OpenGL.dll"
	"Qt5Xml.dll"
	)

