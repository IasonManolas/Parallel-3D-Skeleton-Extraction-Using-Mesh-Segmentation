#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QOpenGLContext>
#include <QOpenGLWidget>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    resize(800,600);
}

MainWindow::~MainWindow()
{
    delete ui;
}
