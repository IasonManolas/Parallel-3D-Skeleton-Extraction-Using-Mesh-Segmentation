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
  ~MainWindow();

private slots:
  void on_checkBoxAxis_clicked(bool state);
  void on_actionOpenFile_triggered();
  void on_checkBoxMeshSurface_clicked(bool state);
  void on_pushButtonResetCamera_clicked();

  void on_checkBoxShowVertices_stateChanged(int arg1);

  void on_actionSave_Model_triggered();

signals:
  void modelLoaded(std::string filename);
  void stateCheckBoxAxesChanged(bool state);
  void stateCheckBoxMeshSurfaceChanged(bool state);
  void buttonResetCameraClicked();
  void stateCheckBoxShowVerticesChanged(int state);
  void actionSaveModelTriggered(std::string destination);

private:
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
