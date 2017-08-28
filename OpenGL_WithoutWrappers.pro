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
    glwidget.cpp \
    meshcontractor.cpp \
    skeleton.cpp \
    mesh.cpp \
    scene.cpp

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
    mesh.h
#    tinyObjLoader/tiny_obj_loader.h

FORMS    += mainwindow.ui

unix:!macx: LIBS += -L$$PWD/'../CGAL build/build/lib/' -lCGAL

INCLUDEPATH += $$PWD/'../CGAL build/build/include'
DEPENDPATH += $$PWD/'../CGAL build/build/include'
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

INCLUDEPATH += $$PWD/../../../../../usr/local/include/eigen3
#DEPENDPATH += $$PWD/../../../../../usr/local/include

#unix:!macx:
LIBS += -L$$PWD/../../../../../usr/local/lib/ -lassimp

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/release/ -lgmp
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/debug/ -lgmp
else:unix: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lgmp
