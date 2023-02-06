cmake_minimum_required(VERSION 3.15)
project(simExtVision)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

find_package(Boost REQUIRED)

if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        set(COPPELIASIM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim 4.5.0.0 REQUIRED)

coppeliasim_add_plugin(
    simExtVision
    LEGACY
    SOURCES
    simExtVision.cpp
    visionCont.cpp
    visionTransf.cpp
    visionTransfCont.cpp
    visionRemap.cpp
    visionRemapCont.cpp
    visionVelodyneHDL64E.cpp
    visionVelodyneHDL64ECont.cpp
    visionVelodyneVPL16.cpp
    visionVelodyneVPL16Cont.cpp
    imageProcess.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MyMath.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/7Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3X3Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4X4Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/MMatrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionData.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionDataItem.cpp
)
target_compile_definitions(simExtVision PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simExtVision PRIVATE ${COPPELIASIM_INCLUDE_DIR}/simMath)
coppeliasim_add_helpfile(simVision.htm)
coppeliasim_add_helpfile(simVision.json SUBDIR index)
