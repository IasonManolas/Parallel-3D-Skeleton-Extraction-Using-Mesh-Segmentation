#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QOpenGLWidget>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QOpenGLWidget *widget=ui->openGLWidget;
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    widget->setFormat(format);

}

MainWindow::~MainWindow()
{
    delete ui;
}
