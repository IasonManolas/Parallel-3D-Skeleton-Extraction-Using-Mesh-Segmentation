#-------------------------------------------------
#
# Project created by QtCreator 2016-10-09T19:48:31
#
#-------------------------------------------------

QT       += core opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenGL_WithoutWrappers
TEMPLATE = app
#CONFIG+=-std=c++1y
QMAKE_CC=clang
QMAKE_CXX=clang

QMAKE_CXXFLAGS+= -std=c++14
#QMAKE_LFLAGS +=  -fopenmp
#LIBS+= -lOpenCL

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
edge.h 

FORMS    += mainwindow.ui

LIBS+=-lboost_system
unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib64/ -lGLEW

INCLUDEPATH += $$PWD/../../../../../usr/include/GL
DEPENDPATH += $$PWD/../../../../../usr/include/GL

DISTFILES += \
   shaders/fragment.glsl \
   shaders/vertex.glsl \
    container.jpg \
    nanosuit.obj \
    Models/default.png \
    Models/copyright.txt \
    Models/default.mtl \
    Icons/Open_folder.png \
    ../../cgal-demos/examples/Surface_mesh_segmentation/data/cactus.off \
    shaders/axesfs.glsl \
    shaders/axesvs.glsl

INCLUDEPATH +=/usr/include/eigen3

#unix:!macx:
LIBS +=  -lassimp

LIBS +=  -lgmp
LIBS+=-lCGAL


INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include
