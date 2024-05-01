cmake_minimum_required(VERSION 3.15)
project(simVision)

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
find_package(CoppeliaSim 4.5.1.7 REQUIRED)

coppeliasim_add_plugin(
    simVision
    LEGACY
    SOURCES
    sourceCode/simVision.cpp
    sourceCode/visionCont.cpp
    sourceCode/visionTransf.cpp
    sourceCode/visionTransfCont.cpp
    sourceCode/visionRemap.cpp
    sourceCode/visionRemapCont.cpp
    sourceCode/visionVelodyneHDL64E.cpp
    sourceCode/visionVelodyneHDL64ECont.cpp
    sourceCode/visionVelodyneVPL16.cpp
    sourceCode/visionVelodyneVPL16Cont.cpp
    sourceCode/imageProcess.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/mathFuncs.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/7Vector.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/3X3Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/4X4Matrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simMath/mXnMatrix.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionData.cpp
    ${COPPELIASIM_INCLUDE_DIR}/simLib/scriptFunctionDataItem.cpp
)
target_compile_definitions(simVision PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simVision PRIVATE ${COPPELIASIM_INCLUDE_DIR}/simMath)
coppeliasim_add_lua(lua/simVision.lua)
coppeliasim_add_lua(lua/simVision-ce.lua)
coppeliasim_add_manual(manual/simVision.htm)
coppeliasim_add_manual(manual/index/simVision.json SUBDIR index)
