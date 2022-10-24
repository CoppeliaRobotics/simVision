QT -= core
QT -= gui

TARGET = simExtVision
TEMPLATE = lib

DEFINES -= UNICODE
DEFINES += QT_COMPIL
CONFIG += shared plugin
INCLUDEPATH += "../include"
INCLUDEPATH += "../simMath"

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
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
    simExtVision.cpp \
    visionCont.cpp \
    visionTransf.cpp \
    visionTransfCont.cpp \
    visionRemap.cpp \
    visionRemapCont.cpp \
    visionVelodyneHDL64E.cpp \
    visionVelodyneHDL64ECont.cpp \
    visionVelodyneVPL16.cpp \
    visionVelodyneVPL16Cont.cpp \
    imageProcess.cpp \
    ../common/scriptFunctionData.cpp \
    ../common/scriptFunctionDataItem.cpp \
    ../common/simLib.cpp \
    ../simMath/MyMath.cpp \
    ../simMath/3Vector.cpp \
    ../simMath/4Vector.cpp \
    ../simMath/7Vector.cpp \
    ../simMath/3X3Matrix.cpp \
    ../simMath/4X4Matrix.cpp \

HEADERS +=\
    simExtVision.h \
    visionCont.h \
    visionTransf.h \
    visionTransfCont.h \
    visionRemap.h \
    visionRemapCont.h \
    visionVelodyneHDL64E.h \
    visionVelodyneHDL64ECont.h \
    visionVelodyneVPL16.h \
    visionVelodyneVPL16Cont.h \
    imageProcess.h \
    ../include/scriptFunctionData.h \
    ../include/scriptFunctionDataItem.h \
    ../include/simLib.h \
    ../simMath/MyMath.h \
    ../simMath/mathDefines.h \
    ../simMath/3Vector.h \
    ../simMath/4Vector.h \
    ../simMath/7Vector.h \
    ../simMath/3X3Matrix.h \
    ../simMath/4X4Matrix.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

