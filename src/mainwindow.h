#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow() { delete ui; }

private slots:
  void on_checkBoxAxis_clicked(bool state);
  void on_actionOpenFile_triggered();
  void on_pushButtonResetCamera_clicked();

  void on_checkBoxShowVertices_stateChanged(int arg1);

  void on_actionSave_Model_triggered();

  void on_actionSave_Segment_triggered();

  void on_contractionThresholdSpinBox_valueChanged(int newThreshold);

  void on_contractionModeCheckBox_stateChanged(int arg1);

  void on_clearSkeletonButton_clicked();

  void on_checkBoxMeshSurface_stateChanged(int arg1);

signals:
  void modelLoaded(std::string filename);
  void stateCheckBoxAxesChanged(bool state);
  void stateCheckBoxMeshSurfaceChanged(int state);
  void buttonResetCameraClicked();
  void stateCheckBoxShowVerticesChanged(int state);
  void actionSaveModelTriggered(std::string destination);
  void actionSaveSegmentTriggered(std::string destination);
  void contractionVolumeThresholdChanged(int newValue);
  void updateContractionMode(bool state);
  void clearSkeleton();

private:
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
