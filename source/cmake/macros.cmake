## macro for adding precompiled header
MACRO (ADD_MSVC_PRECOMPILED_HEADER PrecompiledHeaderBase SourcesVar)
    IF(MSVC)
        SET(PrecompiledHeader "${PrecompiledHeaderBase}.h")
        SET(PrecompiledSource "${PrecompiledHeaderBase}.cpp")
        SET(PrecompiledBinary "${CMAKE_CURRENT_BINARY_DIR}\\${CMAKE_CFG_INTDIR}\\${PrecompiledHeaderBase}.pch")
        IF (${ARGC} GREATER 2)
            SET(PrecompiledBinary ${ARGN})
        ENDIF ()
        #message(${PROJECT_NAME} ": " ${PrecompiledBinary})

        FOREACH( fname ${${SourcesVar}} )
            GET_FILENAME_COMPONENT( filepath ${fname} PATH )
            GET_FILENAME_COMPONENT( filenwe ${fname} NAME_WE )
            IF( filenwe STREQUAL ${PrecompiledHeaderBase} )
                #SET(PrecompiledHeader "${filepath}\\${PrecompiledHeaderBase}.h")
                #message("header name: ${PrecompiledHeader}")
                #SET(PrecompiledSource "${filepath}\\${PrecompiledHeaderBase}.cpp")
                #message("body name: ${PrecompiledSource}") 
            ENDIF()
        ENDFOREACH()

        FOREACH( fname ${${SourcesVar}} )
            GET_FILENAME_COMPONENT( fileext ${fname} EXT )
            GET_FILENAME_COMPONENT( filenwe ${fname} NAME_WE )
            IF( fileext STREQUAL ".cpp" )
                IF( filenwe STREQUAL ${PrecompiledHeaderBase} )
                    SET_SOURCE_FILES_PROPERTIES(${fname}
                        PROPERTIES COMPILE_FLAGS "/Yc\"${PrecompiledHeader}\" /Fp\"${PrecompiledBinary}\""
                        OBJECT_OUTPUTS "${PrecompiledBinary}")
                ELSE()
                    SET_SOURCE_FILES_PROPERTIES(${fname}
                        PROPERTIES COMPILE_FLAGS "/Yu\"${PrecompiledBinary}\" /FI\"${PrecompiledBinary}\" /Fp\"${PrecompiledBinary}\""
                        OBJECT_DEPENDS "${PrecompiledBinary}")
                ENDIF()
            ENDIF()
        ENDFOREACH()
    ENDIF(MSVC)
ENDMACRO (ADD_MSVC_PRECOMPILED_HEADER)

## macro for using precompiled header generated from other target
MACRO (USE_MSVC_PRECOMPILED_HEADER SourcesVar)
    IF(MSVC)
        SET (pch_src "${pch_basename}.cpp")
        #SET (pch_inc "${pch_basename}.h")
        SET (pch_bin "${CMAKE_BINARY_DIR}/common/${CMAKE_CFG_INTDIR}/${pch_basename}.pch")
        SET_SOURCE_FILES_PROPERTIES(${prj_srcs}
            PROPERTIES COMPILE_FLAGS "/Yu\"${pch_bin}\" /FI\"${pch_bin}\" /Fp\"${pch_bin}\""
            OBJECT_DEPENDS "${pch_bin}")
        LIST(APPEND ${SourcesVar} "${pch_path}/${pch_src}")
    ENDIF(MSVC)
ENDMACRO (USE_MSVC_PRECOMPILED_HEADER)

## collect subproject with "CMakeLists.txt"
MACRO (collect_subproject_directory_names dirname filename dirs mods)
    file(GLOB_RECURSE globbed RELATIVE "${dirname}" "${dirname}/*/${filename}")
    if(${ARGC} GREATER 4)
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            foreach(exclusion ${ARGN})
                string(REGEX MATCH ".*(${exclusion}).*" rmvar ${dir})
                string(REGEX REPLACE ".*(${exclusion}).*" "\\1" rmvar "${rmvar}")
                if(rmvar STREQUAL exclusion)
                    break()
                endif()
            endforeach()
            if(rmvar STREQUAL "")
                set(${dirs} ${${dirs}} ${dir})
            endif()
        endforeach()
    else(${ARGC} GREATER 4)
        foreach(file ${globbed})
            get_filename_component(dir ${file} PATH)
            set(${dirs} ${${dirs}} ${dir})
        endforeach(file)
    endif(${ARGC} GREATER 4)
    foreach(subdir ${${dirs}})
        file(STRINGS ${dirname}/${subdir}/CMakeLists.txt mod LIMIT_COUNT 1 REGEX "^PROJECT\\(.*\\)$")
        string(REGEX REPLACE "^PROJECT\\((.*)\\)$" "\\1" mod "${mod}")
        string(STRIP "${mod}" mod)
        set(${mods} ${${mods}} ${mod})
    endforeach(subdir)
ENDMACRO ()

## group subprojects into folders
MACRO (group_subproject_folder folder)
    FOREACH (it ${ARGN})
        SET_PROPERTY(TARGET ${it} PROPERTY FOLDER ${folder})
    ENDFOREACH (it)
ENDMACRO ()

## root directory wrapper
MACRO (collect_subprojects rootdir)
    FILE (GLOB path_list RELATIVE ${rootdir} "${rootdir}/*")
    foreach (pvar ${path_list})
        IF (IS_DIRECTORY ${rootdir}/${pvar})
            SET (MOD_DIR_SUB )
            SET (MOD_NAMES )
            collect_subproject_directory_names (${rootdir}/${pvar} "CMakeLists.txt" MOD_DIR_SUB MOD_NAMES "include" "doc" ${ARGN})
            FOREACH (subdir ${MOD_DIR_SUB})
                ADD_SUBDIRECTORY (${rootdir}/${pvar}/${subdir})
            ENDFOREACH (subdir)
            group_subproject_folder("${pvar}" ${MOD_NAMES})
        ENDIF()
    endforeach()
ENDMACRO ()

##############################################################################
## macro used to create the names of output files preserving relative dirs
macro (QT5_MAKE_OUTPUT_FILE_MOD infile prefix postfix ext outfile)

    get_filename_component(_outfile ${infile} NAME_WE)
    set(${outfile} ${MOC_OUT_DIR}/${prefix}${_outfile}${postfix}.${ext})

endmacro ()

##
macro (QT5_CREATE_MOC_COMMAND_MOD infile outfile moc_flags moc_options)

    get_filename_component(_moc_outfile_name "${outfile}" NAME)
    get_filename_component(_moc_outfile_dir "${outfile}" PATH)
    if(_moc_outfile_dir)
        set(_moc_working_dir WORKING_DIRECTORY ${_moc_outfile_dir})
    endif()
    set(_moc_parameters_file ${outfile}_parameters)
    set(_moc_parameters ${moc_flags} ${moc_options} -o "${outfile}" "${infile}")
    string(REPLACE ";" "\n" _moc_parameters "${_moc_parameters}")
    file(WRITE ${_moc_parameters_file} "${_moc_parameters}")
    # add_custom_command is a build time command, will not produce output during configure time
    #	add_custom_command(OUTPUT ${outfile}
    #					   COMMAND ${Qt5Core_MOC_EXECUTABLE} @${_moc_outfile_name}_parameters
    #					   DEPENDS ${infile}
    #					   ${_moc_working_dir}
    #					   VERBATIM)
    #message(${_moc_working_dir})
    if (NOT EXISTS ${_moc_outfile_name})
        EXECUTE_PROCESS(COMMAND ${QT_MOC_EXECUTABLE} @${_moc_outfile_name}_parameters
            ${_moc_working_dir}
            )
    endif ()

endmacro ()

## wraping headers: *.h -> ${}_moc.h
MACRO (QT5_WRAP_CPP_MOD outfiles)
    # get include dirs
    qt5_get_moc_flags(moc_flags)

    set(options)
    set(oneValueArgs)
    set(multiValueArgs OPTIONS)

    cmake_parse_arguments(_WRAP_CPP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(moc_files ${_WRAP_CPP_UNPARSED_ARGUMENTS})
    set(moc_options ${_WRAP_CPP_OPTIONS})
    foreach(it ${moc_files})
        get_filename_component(it ${it} ABSOLUTE)
        #qt5_make_output_file(${it} moc_ h outfile)
        #qt5_create_moc_command(${it} ${outfile} "${moc_flags}" "${moc_options}")
        QT5_MAKE_OUTPUT_FILE_MOD(${it} "" _moc h outfile)
        QT5_CREATE_MOC_COMMAND_MOD(${it} ${outfile} "${moc_flags}" "${moc_options}")
        list(APPEND ${outfiles} ${outfile})
    endforeach()
    set(${outfiles} ${${outfiles}} PARENT_SCOPE)

ENDMACRO ()

# wraping resource: *.qrc -> *_rcc.h
function (QT5_ADD_RESOURCES_MOD outfiles)

    set(options)
    set(oneValueArgs)
    set(multiValueArgs OPTIONS)

    cmake_parse_arguments(_RCC "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(rcc_files ${_RCC_UNPARSED_ARGUMENTS})
    set(rcc_options ${_RCC_OPTIONS})

    foreach(it ${rcc_files})
        get_filename_component(outfilename ${it} NAME_WE)
        get_filename_component(infile ${it} ABSOLUTE)
        get_filename_component(rc_path ${infile} PATH)
        set(outfile ${MOC_OUT_DIR}/${outfilename}_rcc.h)

        set(_RC_DEPENDS)
        if(EXISTS "${infile}")
            #  parse file for dependencies
            #  all files are absolute paths or relative to the location of the qrc file
            file(READ "${infile}" _RC_FILE_CONTENTS)
            string(REGEX MATCHALL "<file[^<]+" _RC_FILES "${_RC_FILE_CONTENTS}")
            foreach(_RC_FILE ${_RC_FILES})
                string(REGEX REPLACE "^<file[^>]*>" "" _RC_FILE "${_RC_FILE}")
                if(NOT IS_ABSOLUTE "${_RC_FILE}")
                    set(_RC_FILE "${rc_path}/${_RC_FILE}")
                endif()
                set(_RC_DEPENDS ${_RC_DEPENDS} "${_RC_FILE}")
            endforeach()
            # Since this cmake macro is doing the dependency scanning for these files,
            # let's make a configured file and add it as a dependency so cmake is run
            # again when dependencies need to be recomputed.
            qt5_make_output_file("${infile}" "" "qrc.depends" out_depends)
            configure_file("${infile}" "${out_depends}" COPYONLY)
        else()
            # The .qrc file does not exist (yet). Let's add a dependency and hope
            # that it will be generated later
            set(out_depends)
        endif()

        add_custom_command(OUTPUT ${outfile}
            COMMAND ${Qt5Core_RCC_EXECUTABLE}
            #ARGS ${rcc_options} -name ${outfilename} -o ${outfile} ${infile}
            ARGS ${rcc_options} -o ${outfile} ${infile}
            MAIN_DEPENDENCY ${infile}
            DEPENDS ${_RC_DEPENDS} "${out_depends}" VERBATIM)
        list(APPEND ${outfiles} ${outfile})
    endforeach()
    set(${outfiles} ${${outfiles}} PARENT_SCOPE)
endfunction ()

## wraping ui: *.ui -> *${}_ui.h
function (QT5_WRAP_UI_MOD outfiles)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs OPTIONS)

    cmake_parse_arguments(_WRAP_UI "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set(ui_files ${_WRAP_UI_UNPARSED_ARGUMENTS})
    set(ui_options ${_WRAP_UI_OPTIONS})

    foreach(it ${ui_files})
        get_filename_component(outfile ${it} NAME_WE)
        get_filename_component(infile ${it} ABSOLUTE)
        set(outfile ${MOC_OUT_DIR}/${outfile}_ui.h)
        add_custom_command(OUTPUT ${outfile}
            COMMAND ${Qt5Widgets_UIC_EXECUTABLE}
            ARGS ${ui_options} -o ${outfile} ${infile}
            MAIN_DEPENDENCY ${infile} VERBATIM)
        list(APPEND ${outfiles} ${outfile})
    endforeach()
    set(${outfiles} ${${outfiles}} PARENT_SCOPE)
endfunction ()

##############################################################################
## wraping ui: *.ui -> *_ui.h
MACRO (QT4_WRAP_UI_MOD outfiles)
    QT4_EXTRACT_OPTIONS(ui_files ui_options ui_target ${ARGN})

    FOREACH (it ${ui_files})
        GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)
        GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
        #SET(outfile ${MOC_OUT_DIR}/${outfile}_ui.h)
        SET(outfile ${MOC_OUT_DIR}/${outfile}_ui.h)
        ADD_CUSTOM_COMMAND(OUTPUT ${outfile}
            COMMAND ${QT_UIC_EXECUTABLE}
            ARGS ${ui_options} -o ${outfile} ${infile}
            MAIN_DEPENDENCY ${infile})
        SET(${outfiles} ${${outfiles}} ${outfile})
    ENDFOREACH (it)

ENDMACRO (QT4_WRAP_UI_MOD)

# macro for modified output
MACRO (QT4_MAKE_OUTPUT_FILE_MOD infile prefix postfix ext outfile)
    STRING(LENGTH ${CMAKE_CURRENT_BINARY_DIR} _binlength)
    STRING(LENGTH ${infile} _infileLength)
    SET(_checkinfile ${CMAKE_CURRENT_SOURCE_DIR})
    IF(_infileLength GREATER _binlength)
        STRING(SUBSTRING "${infile}" 0 ${_binlength} _checkinfile)
    ENDIF(_infileLength GREATER _binlength)
    IF(CMAKE_CURRENT_BINARY_DIR MATCHES "${_checkinfile}")
        FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_BINARY_DIR} ${infile})
    ELSE(CMAKE_CURRENT_BINARY_DIR MATCHES "${_checkinfile}")
        FILE(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${infile})
    ENDIF(CMAKE_CURRENT_BINARY_DIR MATCHES "${_checkinfile}")
    SET(_outfile "${CMAKE_CURRENT_BINARY_DIR}/${rel}")
    STRING(REPLACE ".." "__" _outfile ${_outfile})
    #GET_FILENAME_COMPONENT(outpath ${_outfile} PATH)
    SET(outpath ${MOC_OUT_DIR})
    GET_FILENAME_COMPONENT(_outfile ${_outfile} NAME_WE)
    FILE(MAKE_DIRECTORY ${outpath})
    SET(${outfile} ${outpath}/${prefix}${_outfile}${postfix}.${ext})
ENDMACRO (QT4_MAKE_OUTPUT_FILE_MOD)

# wraping headers: *.h -> *_moc.h
MACRO (QT4_WRAP_CPP_MOD outfiles)
    # get include dirs
    QT4_GET_MOC_FLAGS(moc_flags)
    QT4_EXTRACT_OPTIONS(moc_files moc_options moc_target ${ARGN})

    FOREACH (it ${moc_files})
        GET_FILENAME_COMPONENT(it ${it} ABSOLUTE)
        #QT4_MAKE_OUTPUT_FILE(${it} moc_ cxx outfile)
        QT4_MAKE_OUTPUT_FILE_MOD(${it} "" _moc h outfile)
        QT4_CREATE_MOC_COMMAND(${it} ${outfile} "${moc_flags}" "${moc_options}" "${moc_target}")
        SET(${outfiles} ${${outfiles}} ${outfile})
    ENDFOREACH(it)

ENDMACRO (QT4_WRAP_CPP_MOD)

# wraping resource: *.qrc -> *_rcc.h
MACRO (QT4_ADD_RESOURCES_MOD outfiles)
    QT4_EXTRACT_OPTIONS(rcc_files rcc_options rcc_target ${ARGN})

    FOREACH (it ${rcc_files})
        GET_FILENAME_COMPONENT(outfilename ${it} NAME_WE)
        GET_FILENAME_COMPONENT(infile ${it} ABSOLUTE)
        GET_FILENAME_COMPONENT(rc_path ${infile} PATH)
        SET(outfile ${MOC_OUT_DIR}/${outfilename}_rcc.h)
        #  parse file for dependencies 
        #  all files are absolute paths or relative to the location of the qrc file
        FILE(READ "${infile}" _RC_FILE_CONTENTS)
        STRING(REGEX MATCHALL "<file[^<]+" _RC_FILES "${_RC_FILE_CONTENTS}")
        SET(_RC_DEPENDS)
        FOREACH(_RC_FILE ${_RC_FILES})
            STRING(REGEX REPLACE "^<file[^>]*>" "" _RC_FILE "${_RC_FILE}")
            STRING(REGEX MATCH "^/|([A-Za-z]:/)" _ABS_PATH_INDICATOR "${_RC_FILE}")
            IF(NOT _ABS_PATH_INDICATOR)
                SET(_RC_FILE "${rc_path}/${_RC_FILE}")
            ENDIF(NOT _ABS_PATH_INDICATOR)
            SET(_RC_DEPENDS ${_RC_DEPENDS} "${_RC_FILE}")
        ENDFOREACH(_RC_FILE)
        ADD_CUSTOM_COMMAND(
            OUTPUT ${outfile}
            COMMAND ${QT_RCC_EXECUTABLE}
            #ARGS ${rcc_options} -name ${outfilename} -o ${outfile} ${infile}
            ARGS ${rcc_options} -o ${outfile} ${infile}
            MAIN_DEPENDENCY ${infile}
            DEPENDS ${_RC_DEPENDS}
            VERBATIM
            )
        SET(${outfiles} ${${outfiles}} ${outfile})
    ENDFOREACH (it)

ENDMACRO (QT4_ADD_RESOURCES_MOD)

##############################################################################
MACRO (QT_WRAP_CPP_MOD outfiles)
    IF (QT_VERSION EQUAL 4)
        QT4_WRAP_CPP_MOD(${outfiles} ${ARGN})
    ELSEIF (QT_VERSION EQUAL 5)
        QT5_WRAP_CPP_MOD(${outfiles} ${ARGN})
    ENDIF ()
ENDMACRO ()

MACRO (QT_WRAP_UI_MOD outfiles)
    IF (QT_VERSION EQUAL 4)
        QT4_WRAP_UI_MOD(${outfiles} ${ARGN})
    ELSEIF (QT_VERSION EQUAL 5)
        QT5_WRAP_UI_MOD(${outfiles} ${ARGN})
    ENDIF ()
ENDMACRO ()

MACRO (QT_ADD_RESOURCES_MOD outfiles)
    IF (QT_VERSION EQUAL 4)
        QT4_ADD_RESOURCES_MOD(${outfiles} ${ARGN})
    ELSEIF (QT_VERSION EQUAL 5)
        QT5_ADD_RESOURCES_MOD(${outfiles} ${ARGN})
    ENDIF ()
ENDMACRO ()
