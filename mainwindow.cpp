#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QOpenGLContext>
#include <QOpenGLWidget>
#include <qdebug.h>

#include <CGAL/Polygon_mesh_processing/measure.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  resize(800, 600);
  connect(this, SIGNAL(modelLoaded(std::string)), ui->openGLWidget,
          SLOT(modelWasChosen(std::string)));
  connect(this, SIGNAL(stateCheckBoxAxesChanged(bool)), ui->openGLWidget,
          SLOT(updateAxesState(bool)));
  connect(this, SIGNAL(stateCheckBoxMeshSurfaceChanged(bool)), ui->openGLWidget,
          SLOT(updateMeshSurfaceState(bool)));
  connect(this, SIGNAL(buttonResetCameraClicked()), ui->openGLWidget,
          SLOT(resetCamera()));
  connect(this, SIGNAL(stateCheckBoxShowVerticesChanged(int)), ui->openGLWidget,
          SLOT(showVerticesStateChange(int)));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_checkBoxAxis_clicked(bool state) {
  emit stateCheckBoxAxesChanged(state);
}

void MainWindow::on_actionOpenFile_triggered() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Load Model"), "../Models/", tr("All Files (*)"));
  if (filename != NULL) {
    std::string filenameString = filename.toUtf8().constData();
    emit modelLoaded(filenameString);
  }
}

void MainWindow::on_checkBoxMeshSurface_clicked(bool state) {
  emit stateCheckBoxMeshSurfaceChanged(state);
}

void MainWindow::on_pushButtonResetCamera_clicked() {
  std::cout << "camera reseted" << std::endl;
  emit buttonResetCameraClicked();
}

void MainWindow::on_checkBoxShowVertices_stateChanged(int arg1) {
  emit stateCheckBoxShowVerticesChanged(arg1);
}
