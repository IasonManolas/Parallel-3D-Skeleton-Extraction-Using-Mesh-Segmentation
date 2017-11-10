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
          SLOT(loadModel(std::string)));
  connect(this, SIGNAL(stateCheckBoxAxesChanged(bool)), ui->openGLWidget,
          SLOT(updateAxesState(bool)));
  connect(this, SIGNAL(stateCheckBoxMeshSurfaceChanged(bool)), ui->openGLWidget,
          SLOT(updateMeshSurfaceState(bool)));
  connect(this, SIGNAL(buttonResetCameraClicked()), ui->openGLWidget,
          SLOT(resetCamera()));
  connect(this, SIGNAL(stateCheckBoxShowVerticesChanged(int)), ui->openGLWidget,
          SLOT(showVerticesStateChange(int)));
  connect(this, SIGNAL(actionSaveModelTriggered(std::string)), ui->openGLWidget,
          SLOT(saveModel(std::string)));
  connect(this, SIGNAL(actionSaveSegmentTriggered(std::string)),
          ui->openGLWidget, SLOT(saveSegment(std::string)));
  connect(this,SIGNAL(contractionVolumeThresholdChanged(int)),ui->openGLWidget,SLOT(updateContractionVolumeThreshold(int)));
  connect(this,SIGNAL(updateContractionMode(bool)),ui->openGLWidget,SLOT(updateContractionMode(bool)));
  connect(this,SIGNAL(clearSkeleton()),ui->openGLWidget,SLOT(clearSkeleton()));
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_checkBoxAxis_clicked(bool state) {
  emit stateCheckBoxAxesChanged(state);
}

void MainWindow::on_actionOpenFile_triggered() {
  QString filename = QFileDialog::getOpenFileName(
      this, tr("Load Model"), "../Models/", tr("Object files(*.obj *off)"));
  if (filename != NULL) {
    std::string filenameString = filename.toUtf8().constData();
    emit modelLoaded(filenameString);
  }
  // uncheck show vertices when model is loaded
  // ui->checkBoxShowVertices->setChecked(false);
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

void MainWindow::on_actionSave_Model_triggered() {
  QString destinationDirectory = QFileDialog::getSaveFileName(
      this, tr("Save Model"), "../Models/", tr("Object file(*.off)"));
  if (destinationDirectory != NULL) {
    std::string destinationDirectory_std =
        destinationDirectory.toUtf8().constData();
    emit actionSaveModelTriggered(destinationDirectory_std);
  }
}

void MainWindow::on_actionSave_Segment_triggered() {
  QString destinationDirectory = QFileDialog::getSaveFileName(
      this, tr("Save Model"), "../Models/", tr("Object file(*.off)"));
  if (destinationDirectory != NULL) {
    std::string destinationDirectory_std =
        destinationDirectory.toUtf8().constData();
    emit actionSaveSegmentTriggered(destinationDirectory_std);
  }
}

void MainWindow::on_contractionThresholdSpinBox_valueChanged(int newThreshold)
{
    emit contractionVolumeThresholdChanged(newThreshold);

}


void MainWindow::on_contractionModeCheckBox_stateChanged(int arg1)
{
   emit updateContractionMode(bool(arg1));
}

void MainWindow::on_clearSkeletonButton_clicked()
{
   emit clearSkeleton();
}
