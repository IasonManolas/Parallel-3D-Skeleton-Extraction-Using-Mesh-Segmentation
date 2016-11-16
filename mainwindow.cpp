#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QOpenGLContext>
#include <QOpenGLWidget>
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    resize(800,600);
    connect(this,SIGNAL(modelLoaded(std::string)),ui->openGLWidget,SLOT(modelWasChosen(std::string)));
    connect(this,SIGNAL(stateCheckBoxAxesChanged(bool)),ui->openGLWidget,SLOT(updateAxesState(bool)));
    connect(this,SIGNAL(stateCheckBoxMeshSurfaceChanged(bool)),ui->openGLWidget,SLOT(updateMeshSurfaceState(bool)));
    connect(this,SIGNAL(buttonResetCameraClicked()),ui->openGLWidget,SLOT(resetCamera()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_checkBoxAxis_clicked(bool state)
{
    emit stateCheckBoxAxesChanged(state);
}

void MainWindow::on_actionOpenFile_triggered()
{
    QString filename=QFileDialog::getOpenFileName(this,tr("Load Model"),"../OpenGL_WithoutWrappers/Models/",tr("All Files (*)"));
    if(filename!=NULL)
    {
        std::string filenameString=filename.toUtf8().constData();
        emit modelLoaded(filenameString);
//       ui->openGLWidget->scene.mesh.setupDrawingBuffers();
    }
}

void MainWindow::on_checkBoxMeshSurface_clicked(bool state)
{
    emit stateCheckBoxMeshSurfaceChanged(state);
}

void MainWindow::on_pushButtonResetCamera_clicked()
{
    emit buttonResetCameraClicked();
}
