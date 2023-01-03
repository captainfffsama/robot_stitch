DEFINES += ROBOT_STITCH_LIBRARY

CONFIG += c++17

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0




#DEFINES += UNIT_TEST


if(contains(DEFINES,UNIT_TEST)){
    TEMPLATE = app
    SOURCES += \
        robotimagestitch.cpp \
        unit_test/test.cpp

    HEADERS += \
        robot_stitch_global.h \
        robotimagestitch.h
}else{
    TEMPLATE = lib
    SOURCES += \
        robotimagestitch.cpp
    HEADERS += \
        robot_stitch_global.h \
        robotimagestitch.h
}

include($(PROJECTSRC)/base/pri/grpc.pri)
include($(PROJECTSRC)/base/pri/opencv.pri)

win32-msvc* {
    QMAKE_CXXFLAGS += /source-charset:utf-8 /execution-charset:utf-8
}

HEADERS += \
    alignrpc.h \
    kp2d.grpc.pb.h \
    kp2d.pb.h

SOURCES += \
    alignrpc.cpp \
    kp2d.grpc.pb.cc \
    kp2d.pb.cc



