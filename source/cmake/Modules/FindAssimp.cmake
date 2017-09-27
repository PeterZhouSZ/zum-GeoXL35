# - Try to find Assimp
# Once done this will define
#
#  ASSIMP_FOUND - system has Assimp
#  ASSIMP_INCLUDE - the Assimp include directory
#  ASSIMP_LIBRARY - Link these to use Assimp
#  ASSIMP_PATH - path holding Assimp
#  ASSIMP_BIN - binary file

set(_assimp_INCLUDE_SEARCH_DIRS_SYSTEM
    C:/assimp/include
    C:/assimp
    "$ENV{ProgramFiles}/assimp/include"
    "$ENV{ProgramFiles}/assimp"
    /sw/local/include
    )

set(_assimp_LIB_SEARCH_DIRS_SYSTEM
    C:/assimp/lib
    C:/assimp
    "$ENV{ProgramFiles}/assimp/lib"
    "$ENV{ProgramFiles}/assimp"
    /sw/local/lib
    )

FIND_PATH(ASSIMP_INCLUDE assimp/assimp.hpp
    $ENV{ASSIMPSDIR}/include
    $ENV{ASSIMPSDIR}
    $ENV{ASSIMPSDIR}/..
    /usr/local/include/assimp
    /usr/local/include
    /usr/include/assimp
    /usr/include
    /sw/include/assimp # Fink
    /sw/include
    /opt/local/include/assimp # DarwinPorts
    /opt/local/include
    /opt/csw/include/assimp # Blastwave
    /opt/csw/include
    /opt/include/assimp
    /opt/include
    ${_assimp_INCLUDE_SEARCH_DIRS_SYSTEM}
    ${CMAKE_SOURCE_DIR}/thirdParty/Assimp/include
    )
IF(ASSIMP_INCLUDE)
	get_filename_component(ASSIMP_PATH ${ASSIMP_INCLUDE} DIRECTORY)
ENDIF()

FIND_LIBRARY(ASSIMP_LIBRARY
    NAMES assimp
    PATHS
    $ENV{ASSIMPSDIR}/lib
    /usr/local/lib
    /usr/lib
    /sw/lib
    /opt/local/lib
    /opt/csw/lib
    /opt/lib
    ${_assimp_LIB_SEARCH_DIRS_SYSTEM}
    ${CMAKE_SOURCE_DIR}/thirdParty/Assimp/lib
    )

IF (WIN32)
SET(ASSIMP_BIN assimp.dll)
ENDIF ()

SET(ASSIMP_FOUND "NO")
IF(ASSIMP_INCLUDE AND ASSIMP_LIBRARY AND ASSIMP_PATH)
    SET(ASSIMP_FOUND "YES")
    SET(ASSIMP_LIBRARY debug ${ASSIMP_LIBRARY} optimized ${ASSIMP_LIBRARY})
ENDIF()

if(Assimp_FIND_REQUIRED AND NOT (ASSIMP_LIBRARY AND ASSIMP_INCLUDE))
    MESSAGE(FATAL_ERROR "Could not find assimp")
ELSE ()
	MESSAGE(STATUS "using Assimp")
ENDIF(Assimp_FIND_REQUIRED AND NOT (ASSIMP_LIBRARY AND ASSIMP_INCLUDE))
