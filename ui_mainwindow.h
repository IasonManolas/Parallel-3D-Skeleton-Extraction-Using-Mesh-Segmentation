/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtWidgets/QCheckBox>
#include <glwidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayoutCheckBoxes;
    GLWidget *openGLWidget;
    QPushButton *pushButtonResetCamera;
    QCheckBox *checkBoxAxis;
    QCheckBox *checkBoxMeshSurface;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QAction *actionOpenFile;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(400, 300);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayoutCheckBoxes=new QHBoxLayout();
        horizontalLayoutCheckBoxes->setObjectName("horizontalBoxLayoutCheckBoxes");
        openGLWidget = new GLWidget(centralWidget);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));
        actionOpenFile=new QAction(MainWindow);
        actionOpenFile->setObjectName(QStringLiteral("actionOpenFile"));
        actionOpenFile->setEnabled(true);
        QIcon iconOpenFile;
        iconOpenFile.addFile("../OpenGL_WithoutWrappers/Icons/Open_folder.png");
        actionOpenFile->setIcon(iconOpenFile);
        verticalLayout->addWidget(openGLWidget);

        checkBoxAxis=new QCheckBox(centralWidget);
        checkBoxAxis->setObjectName(QStringLiteral("checkBoxAxis"));
        checkBoxAxis->setText("Axis");
        horizontalLayoutCheckBoxes->addWidget(checkBoxAxis);


        checkBoxMeshSurface=new QCheckBox(centralWidget);
        checkBoxMeshSurface->setObjectName(QStringLiteral("checkBoxMeshSurface"));
        checkBoxMeshSurface->setText("Show Grid");
//        checkBoxMeshSurface->setState(Qt::Checked);
        horizontalLayoutCheckBoxes->addWidget(checkBoxMeshSurface);

        pushButtonResetCamera=new QPushButton(centralWidget);
        pushButtonResetCamera->setObjectName(QStringLiteral("pushButtonResetCamera"));
        pushButtonResetCamera->setText("Reset Camera");
        horizontalLayoutCheckBoxes->addWidget(pushButtonResetCamera);
        
        verticalLayout->addLayout(horizontalLayoutCheckBoxes);


        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 400, 19));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        mainToolBar->addAction(actionOpenFile);
        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionOpenFile->setText(QApplication::translate("MainWindow","Open File..",0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
