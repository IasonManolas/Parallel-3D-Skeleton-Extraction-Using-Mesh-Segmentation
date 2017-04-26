#-------------------------------------------------
#
# Project created by QtCreator 2016-10-09T19:48:31
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenGL_WithoutWrappers
TEMPLATE = app
CONFIG+=c++14

QMAKE_CC=clang
QMAKE_CXX=clang

SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp

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
    mypolyhedron.h \
    meshloader.h \
    ray_cast_picking.h \
    pointSphere.h \
    pointsphere.h

FORMS    += mainwindow.ui

LIBS+=-lboost_system
#LIBS+=-lCGAL
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

#INCLUDEPATH += $$PWD/../../../../../usr/local/include
#DEPENDPATH += $$PWD/../../../../../usr/local/include

#unix:!macx:
LIBS += -L$$PWD/../../../../../usr/local/lib/ -lassimp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/release/ -lgmp
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/debug/ -lgmp
else:unix: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lgmp

#INCLUDEPATH += $$PWD/../../../../../usr/local/include
#DEPENDPATH += $$PWD/../../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/'../../CGAL build/build/debug/lib/' -lCGAL

INCLUDEPATH += $$PWD/'../../CGAL build'
#DEPENDPATH += $$PWD/'../../CGAL build'
