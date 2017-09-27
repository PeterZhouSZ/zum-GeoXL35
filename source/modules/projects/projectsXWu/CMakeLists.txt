PROJECT(projectsXWu)
IF (USE_OPENCV)
SET (PROJECT_DEPS
	x3editor x3editor2 x3foundation_private x3geometry x3openGL x3rendering x3sceneGraph x4basics x4core x4interface x4math x4miscFoundation x4modules x4tools x3deformation x3symmetryToolBox
	${OpenCV_LIBS}
	)
ELSE ()
SET (PROJECT_DEPS
	x3editor x3editor2 x3foundation_private x3geometry x3openGL x3rendering x3sceneGraph x4basics x4core x4interface x4math x4miscFoundation x4modules x4tools x3deformation x3symmetryToolBox
	)
ENDIF ()


INCLUDE_DIRECTORIES(
	${Eigen_INCLUDE}
	${CG_INCLUDE_PATH}
	)
	
IF (USE_OPENCV)
INCLUDE_DIRECTORIES(
	${OpenCV_INCLUDE_DIRS}
	)
ENDIF ()

## CUDA
IF (${USE_CUDA})
    SET (PROJECT_DEPS ${PROJECT_DEPS}
        ${CUDA_CUDART_LIBRARY} ${CUDA_CUDA_LIBRARY}
        ${CUDA_cublas_LIBRARY} ${CUDA_cusparse_LIBRARY}
        )
    INCLUDE_DIRECTORIES(
        ${CUDA_TOOLKIT_INCLUDE}
        )
ENDIF()

IF(WIN32)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /DCVT_MULTITHREAD ")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /DCVT_MULTITHREAD ")
ELSE(WIN32)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCVT_MULTITHREAD ")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCVT_MULTITHREAD ")
ENDIF(WIN32)

ADD_LIB_PROJ ("SHARED" "STD"
	"example" "LpCVT" "Eigen"
	)

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Zm125")
#ADD_DEPENDENCIES(${PROJECT_NAME} ${ModuleParserProject})

if (0)
## cgal dll
SET (CGAL_DLL_Core "CGAL")
FOREACH (dll_file ${CGAL_DLL_Core})
    FILE (GLOB inFiles RELATIVE "${CGAL_DIR}/bin"
        "${CGAL_DIR}/bin/${dll_file}-vc*mt-gd*.dll")
    ADD_CUSTOM_COMMAND(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CGAL_DIR}/bin/${inFiles}"
        "${EXECUTABLE_OUTPUT_PATH_DEBUG}/${inFiles}"
        VERBATIM
        )
ENDFOREACH ()
FOREACH (dll_file ${CGAL_DLL_Core})
    FILE (GLOB inFiles RELATIVE "${CGAL_DIR}/bin"
        "${CGAL_DIR}/bin/${dll_file}-vc*mt-[1-9]*.dll")
    ADD_CUSTOM_COMMAND(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CGAL_DIR}/bin/${inFiles}"
        "${EXECUTABLE_OUTPUT_PATH_RELEASE}/${inFiles}"
        VERBATIM
        )
ENDFOREACH ()
FOREACH (dll_file ${CGAL_DLL_Core})
    FILE (GLOB inFiles RELATIVE "${CGAL_DIR}/bin"
        "${CGAL_DIR}/bin/${dll_file}-vc*mt-[1-9]*.dll")
    ADD_CUSTOM_COMMAND(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CGAL_DIR}/bin/${inFiles}"
        "${EXECUTABLE_OUTPUT_PATH_RelWithDebInfo}/${inFiles}"
        VERBATIM
        )
ENDFOREACH ()
endif()
