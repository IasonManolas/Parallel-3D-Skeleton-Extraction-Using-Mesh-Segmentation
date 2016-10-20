#-------------------------------------------------
#
# Project created by QtCreator 2016-10-09T19:48:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenGL_WithoutWrappers
TEMPLATE = app
CONFIG+=c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    glwidget.cpp \
    mesh.cpp \
    model.cpp

HEADERS  += mainwindow.h \
    glwidget.h \
    shader.h \
    ui_mainwindow.h \
    camera.h \
    mesh.h \
    model.h \
    directionallight.h \
    light.h \
    material.h \
    polyhedronbuilder.h

FORMS    += mainwindow.ui

LIBS+=-lCGAL
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
    Models/default.mtl


#win32:config(release, debug|release): libs += -l$$pwd/../../../../../usr/local/lib/release/ -lsoil
#else:win32:config(debug, debug|release): libs += -l$$pwd/../../../../../usr/local/lib/debug/ -lsoil
#else:unix:
 LIBS += -L$$PWD/../../../../../usr/local/lib/ -lSOIL

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

#win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/release/libSOIL.a
#else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/debug/libSOIL.a
#else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/release/SOIL.lib
#else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/debug/SOIL.lib
#else:unix:
PRE_TARGETDEPS += $$PWD/../../../../../usr/local/lib/libSOIL.a

#unix:!macx:
LIBS += -L$$PWD/../../../../../usr/local/lib/ -lassimp
