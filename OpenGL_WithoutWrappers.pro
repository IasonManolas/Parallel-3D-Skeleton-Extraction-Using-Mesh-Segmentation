#-------------------------------------------------
#
# Project created by QtCreator 2016-10-09T19:48:31
#
#-------------------------------------------------

QT       += core opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenGL_WithoutWrappers
TEMPLATE = app

QMAKE_CXXFLAGS+= -std=c++14
QMAKE_CXX = clang++
CONFIG+=release


SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp \
    meshcontractor.cpp \
    mesh.cpp \
    scene.cpp \
    connectivitysurgeon.cpp

HEADERS  += mainwindow.h \
    glwidget.h \
    shader.h \
    ui_mainwindow.h \
    camera.h \
    directionallight.h \
    light.h \
    material.h \
    scene.h \
    axes.h \
    meshloader.h \
    pointSphere.h \
    pointsphere.h \
    cgaltypedefs.h \
    meshsegment.h \
    meshcontractor.h \
    skeleton.h \
    mesh.h \
    connectivitysurgeon.h \
    debug_meshcontractor.h \
    drawablemesh.h \
    polygonalmesh.h \
    drawableskeleton.h \
    undirectedgraph.h \
    linesegments.h \
refinementembedding.h \
edge.h \
node.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib64/ -lGLEW


DISTFILES += \
   shaders/fragment.glsl \
   shaders/vertex.glsl \
    Icons/Open_folder.png \
    shaders/axesfs.glsl \
    shaders/axesvs.glsl

INCLUDEPATH +=/usr/include/eigen3

#unix:!macx:
LIBS +=  -lassimp
LIBS+= /usr/local/lib/libCGAL.so
LIBS+= /usr/lib/x86_64-linux-gnu/libgmp.so
LIBS+=/usr/lib/x86_64-linux-gnu/libmpfr.so
#LIBS+=/usr/lib/x86_64-linux-gnu/libboost_thread.so
#LIBS+=/usr/lib/x86_64-linux-gnu/libboost_system.so


INCLUDEPATH += /usr/local/include

INCLUDEPATH+=/usr/include


