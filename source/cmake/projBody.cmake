## macro for automatically generate "init" file from "tags" file
MACRO (GEN_INIT_FROM_TAGS target infile outfile)
    #ADD_CUSTOM_COMMAND(
    #    TARGET ${target}
    #    PRE_BUILD
    #    COMMAND "x4ModuleParser.exe"
    #    ARGS "-createClassReg" "${infile}" "${outfile}"
    #    WORKING_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
    #    VERBATIM
    #    )
    ADD_CUSTOM_COMMAND(
        OUTPUT ${outfile}
        COMMAND ${ModuleParserProgram}
        ARGS "-createClassReg" "${infile}" "${outfile}"
        WORKING_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
        DEPENDS "${infile}" ${ModuleParserProgram}
        VERBATIM
        )
ENDMACRO ()

## macro for automatically generate "info" file from "tags" file
MACRO (GEN_INFO_FROM_TAGS target infile outfile)
    #ADD_CUSTOM_COMMAND(
    #    TARGET ${target}
    #    POST_BUILD
    #    COMMAND "x4ModuleParser.exe"
    #    ARGS "-createTagsInfo" "${infile}" "${outfile}"
    #    WORKING_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
    #    VERBATIM
    #    )
    ADD_CUSTOM_COMMAND(
        OUTPUT ${outfile}
        COMMAND ${ModuleParserProgram}
        ARGS "-createTagsInfo" "${infile}" "${outfile}"
        WORKING_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}"
        DEPENDS "${infile}" ${ModuleParserProgram}
        VERBATIM
        )
ENDMACRO ()

## template for adding a exe target
MACRO (ADD_EXE_PROJ WIN32_EXE PCH_TYPE)
    STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
    #SET(TARGET_NAME "${PROJECT_NAME}")

    SET(MOC_OUT_DIR "${CMAKE_BINARY_DIR}/moc/${PROJECT_NAME}")
    FILE(MAKE_DIRECTORY ${MOC_OUT_DIR})
    INCLUDE_DIRECTORIES(
        ${module_dir}
        ${CMAKE_CURRENT_SOURCE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/source"
        ${CMAKE_CURRENT_BINARY_DIR}
        "${CMAKE_BINARY_DIR}/moc"
        ${MOC_OUT_DIR}
        )
    SET(DEP_LIST ${PROJECT_DEPS} ${PROJECT_NAME})
    FILE(READ "${LIST_INCS_CMAKE}" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    FOREACH( dir_path ${contents} )
        foreach(depname ${DEP_LIST})
            STRING(FIND "${dir_path}" "${depname}" EC_Pos)
            IF (NOT (${EC_Pos} EQUAL -1))
                INCLUDE_DIRECTORIES(${dir_path})
            ENDIF ()
        endforeach()
    ENDFOREACH()
    LIST_FILE( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN} )

    IF (${PCH_TYPE} STREQUAL "CREATE")
        ADD_MSVC_PRECOMPILED_HEADER( ${pch_basename} prj_srcs
           "${CMAKE_BINARY_DIR}/common/${CMAKE_CFG_INTDIR}/${pch_basename}.pch" )
    ELSEIF (${PCH_TYPE} STREQUAL "USE")
        USE_MSVC_PRECOMPILED_HEADER( prj_srcs )
    ELSEIF (${PCH_TYPE} STREQUAL "NO")
    ELSEIF (${PCH_TYPE} STREQUAL "STD")
        ADD_MSVC_PRECOMPILED_HEADER( ${pch_std_name} prj_srcs )
    ENDIF ()

    ADD_EXECUTABLE(${PROJECT_NAME} ${WIN32_EXE}
        ${prj_srcs} ${prj_incs} ${uic_srcs} ${rcc_incs} ${moc_incs})
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${PROJECT_DEPS}
        )

    ADD_CUSTOM_COMMAND(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/doc"
        "${X_DOC_DIR}"
        VERBATIM
        )

ENDMACRO (ADD_EXE_PROJ)

## template for adding a lib target
# STATIC | SHARED | MODULE
MACRO (ADD_LIB_PROJ LIB_TYPE PCH_TYPE)
    STRING(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
    SET(TARGET_NAME "${PROJECT_NAME}.x4module")
    ADD_DEFINITIONS(-D${PROJECT_NAME_UPPER}_EXPORTS)

    SET(MOC_OUT_DIR "${CMAKE_BINARY_DIR}/moc/${PROJECT_NAME}")
    FILE(MAKE_DIRECTORY ${MOC_OUT_DIR})
    INCLUDE_DIRECTORIES(
        ${module_dir}
        ${CMAKE_CURRENT_SOURCE_DIR}
        "${CMAKE_CURRENT_SOURCE_DIR}/source"
        ${CMAKE_CURRENT_BINARY_DIR}
        "${CMAKE_BINARY_DIR}/moc"
        ${MOC_OUT_DIR}
        )
    SET(DEP_LIST ${PROJECT_DEPS} ${PROJECT_NAME})
    FILE(READ "${LIST_INCS_CMAKE}" contents)
    STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
    STRING(REGEX REPLACE "\n" ";" contents "${contents}")
    FOREACH( dir_path ${contents} )
        foreach(depname ${DEP_LIST})
            STRING(FIND "${dir_path}" "${depname}" EC_Pos)
            IF (NOT (${EC_Pos} EQUAL -1))
                INCLUDE_DIRECTORIES(${dir_path})
            ENDIF ()
        endforeach()
    ENDFOREACH()
    LIST_FILE( ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN} )

    IF (${PCH_TYPE} STREQUAL "CREATE")
        ADD_MSVC_PRECOMPILED_HEADER( ${pch_basename} prj_srcs
           "${CMAKE_BINARY_DIR}/common/${CMAKE_CFG_INTDIR}/${pch_basename}.pch" )
    ELSEIF (${PCH_TYPE} STREQUAL "USE")
        USE_MSVC_PRECOMPILED_HEADER( prj_srcs )
    ELSEIF (${PCH_TYPE} STREQUAL "NO")
    ELSEIF (${PCH_TYPE} STREQUAL "STD")
        ADD_MSVC_PRECOMPILED_HEADER( ${pch_std_name} prj_srcs )
    ENDIF ()

    SET (${PROJECT_NAME}_init_cpp "${MOC_OUT_DIR}/${PROJECT_NAME}.init.cpp")
    IF (${PCH_TYPE} STREQUAL "STD")
        foreach(srcs ${prj_incs})
            string(REGEX MATCH ".*\\.tags\\.h$" tags_name "${srcs}")
            if(tags_name)
                GEN_INIT_FROM_TAGS (
                    ${PROJECT_NAME}
                    "${CMAKE_CURRENT_SOURCE_DIR}/${tags_name}"
                    ${${PROJECT_NAME}_init_cpp}
                    )
                GEN_INFO_FROM_TAGS (
                    ${PROJECT_NAME}
                    "${CMAKE_CURRENT_SOURCE_DIR}/${tags_name}"
                    "${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.tags.info"
                    )
                break()
            endif()
        endforeach()
        foreach(srcs ${prj_srcs})
            string(REGEX MATCH ".*${PROJECT_NAME}.cpp" prj_cpp_name "${srcs}")
            if(prj_cpp_name)
                SET_PROPERTY(SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/${prj_cpp_name}"
                    APPEND PROPERTY OBJECT_DEPENDS ${${PROJECT_NAME}_init_cpp})
                break()
            endif()
        endforeach()
    ENDIF ()

    ADD_LIBRARY(${PROJECT_NAME} ${LIB_TYPE}
        ${prj_srcs} ${prj_incs} ${uic_srcs} ${rcc_incs} ${moc_incs})
    TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${PROJECT_DEPS}
        ${CMAKE_DL_LIBS}
        )
    SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${TARGET_NAME})
	IF (QT_VERSION EQUAL 5)
	ENDIF ()

    ADD_CUSTOM_COMMAND(
        TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/doc"
        "${X_DOC_DIR}"
        VERBATIM
        )

ENDMACRO (ADD_LIB_PROJ)

