QT -= core
QT -= gui

TARGET = simVision
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += SIM_MATH_DOUBLE # for double-precision
DEFINES += QT_COMPIL
CONFIG += shared plugin
INCLUDEPATH += "../include"

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -fvisibility=hidden
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}


win32 {
    DEFINES += WIN_SIM
}

macx {
    DEFINES += MAC_SIM
}

unix:!macx {
    DEFINES += LIN_SIM
}

SOURCES += \
    sourceCode/simVision.cpp \
    sourceCode/visionCont.cpp \
    sourceCode/visionTransf.cpp \
    sourceCode/visionTransfCont.cpp \
    sourceCode/visionRemap.cpp \
    sourceCode/visionRemapCont.cpp \
    sourceCode/visionVelodyneHDL64E.cpp \
    sourceCode/visionVelodyneHDL64ECont.cpp \
    sourceCode/visionVelodyneVPL16.cpp \
    sourceCode/visionVelodyneVPL16Cont.cpp \
    sourceCode/imageProcess.cpp \
    ../include/simLib/scriptFunctionData.cpp \
    ../include/simLib/scriptFunctionDataItem.cpp \
    ../include/simLib/simLib.cpp \
    ../include/simMath/mathFuncs.cpp \
    ../include/simMath/3Vector.cpp \
    ../include/simMath/4Vector.cpp \
    ../include/simMath/7Vector.cpp \
    ../include/simMath/3X3Matrix.cpp \
    ../include/simMath/4X4Matrix.cpp \
    ../include/simMath/mXnMatrix.cpp \

HEADERS +=\
    sourceCode/simVision.h \
    sourceCode/vis.h \
    sourceCode/visionCont.h \
    sourceCode/visionTransf.h \
    sourceCode/visionTransfCont.h \
    sourceCode/visionRemap.h \
    sourceCode/visionRemapCont.h \
    sourceCode/visionVelodyneHDL64E.h \
    sourceCode/visionVelodyneHDL64ECont.h \
    sourceCode/visionVelodyneVPL16.h \
    sourceCode/visionVelodyneVPL16Cont.h \
    sourceCode/imageProcess.h \
    ../include/simLib/scriptFunctionData.h \
    ../include/simLib/scriptFunctionDataItem.h \
    ../include/simLib/simLib.h \
    ../include/simMath/mathFuncs.h \
    ../include/simMath/mathDefines.h \
    ../include/simMath/3Vector.h \
    ../include/simMath/4Vector.h \
    ../include/simMath/7Vector.h \
    ../include/simMath/3X3Matrix.h \
    ../include/simMath/4X4Matrix.h \
    ../include/simMath/mXnMatrix.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
