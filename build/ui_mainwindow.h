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
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <glwidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpenFile;
    QAction *actionSave_Model;
    QAction *actionSave_Segment;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    GLWidget *openGLWidget;
    QHBoxLayout *horizontalLayout;
    QCheckBox *checkBoxShowVertices;
    QCheckBox *checkBoxMeshSurface;
    QCheckBox *checkBoxAxis;
    QPushButton *pushButtonResetCamera;
    QToolBar *mainToolBar;
    QMenuBar *menuBar;
    QMenu *menuSave;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(437, 310);
        actionOpenFile = new QAction(MainWindow);
        actionOpenFile->setObjectName(QStringLiteral("actionOpenFile"));
        QIcon icon;
        icon.addFile(QStringLiteral("../Icons/Open_folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpenFile->setIcon(icon);
        actionSave_Model = new QAction(MainWindow);
        actionSave_Model->setObjectName(QStringLiteral("actionSave_Model"));
        actionSave_Segment = new QAction(MainWindow);
        actionSave_Segment->setObjectName(QStringLiteral("actionSave_Segment"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        openGLWidget = new GLWidget(centralWidget);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));

        verticalLayout->addWidget(openGLWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        checkBoxShowVertices = new QCheckBox(centralWidget);
        checkBoxShowVertices->setObjectName(QStringLiteral("checkBoxShowVertices"));

        horizontalLayout->addWidget(checkBoxShowVertices);

        checkBoxMeshSurface = new QCheckBox(centralWidget);
        checkBoxMeshSurface->setObjectName(QStringLiteral("checkBoxMeshSurface"));
        checkBoxMeshSurface->setEnabled(true);
        checkBoxMeshSurface->setChecked(false);

        horizontalLayout->addWidget(checkBoxMeshSurface);

        checkBoxAxis = new QCheckBox(centralWidget);
        checkBoxAxis->setObjectName(QStringLiteral("checkBoxAxis"));

        horizontalLayout->addWidget(checkBoxAxis);

        pushButtonResetCamera = new QPushButton(centralWidget);
        pushButtonResetCamera->setObjectName(QStringLiteral("pushButtonResetCamera"));
        pushButtonResetCamera->setMaximumSize(QSize(149, 16777215));

        horizontalLayout->addWidget(pushButtonResetCamera);


        verticalLayout->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 437, 25));
        menuSave = new QMenu(menuBar);
        menuSave->setObjectName(QStringLiteral("menuSave"));
        MainWindow->setMenuBar(menuBar);

        mainToolBar->addSeparator();
        mainToolBar->addAction(actionOpenFile);
        menuBar->addAction(menuSave->menuAction());
        menuSave->addAction(actionSave_Model);
        menuSave->addAction(actionSave_Segment);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        actionOpenFile->setText(QApplication::translate("MainWindow", "OpenFile", 0));
#ifndef QT_NO_TOOLTIP
        actionOpenFile->setToolTip(QApplication::translate("MainWindow", "Open File..", 0));
#endif // QT_NO_TOOLTIP
        actionSave_Model->setText(QApplication::translate("MainWindow", "Save Model", 0));
        actionSave_Segment->setText(QApplication::translate("MainWindow", "Save Segment", 0));
        checkBoxShowVertices->setText(QApplication::translate("MainWindow", "Show Vertices", 0));
        checkBoxMeshSurface->setText(QApplication::translate("MainWindow", "Show Grid", 0));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0));
        pushButtonResetCamera->setText(QApplication::translate("MainWindow", "Reset Camera", 0));
        menuSave->setTitle(QApplication::translate("MainWindow", "Save", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
