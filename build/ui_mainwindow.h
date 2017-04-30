/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <glwidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpenFile;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    GLWidget *openGLWidget;
    QHBoxLayout *horizontalLayout;
    QCheckBox *checkBoxMeshSurface;
    QCheckBox *checkBoxAxis;
    QPushButton *pushButtonResetCamera;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(416, 310);
        actionOpenFile = new QAction(MainWindow);
        actionOpenFile->setObjectName(QString::fromUtf8("actionOpenFile"));
        QIcon icon;
        icon.addFile(QString::fromUtf8("Icons/Open_folder.png"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpenFile->setIcon(icon);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        openGLWidget = new GLWidget(centralWidget);
        openGLWidget->setObjectName(QString::fromUtf8("openGLWidget"));

        verticalLayout->addWidget(openGLWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        checkBoxMeshSurface = new QCheckBox(centralWidget);
        checkBoxMeshSurface->setObjectName(QString::fromUtf8("checkBoxMeshSurface"));
        checkBoxMeshSurface->setEnabled(true);
        checkBoxMeshSurface->setChecked(false);

        horizontalLayout->addWidget(checkBoxMeshSurface);

        checkBoxAxis = new QCheckBox(centralWidget);
        checkBoxAxis->setObjectName(QString::fromUtf8("checkBoxAxis"));

        horizontalLayout->addWidget(checkBoxAxis);

        pushButtonResetCamera = new QPushButton(centralWidget);
        pushButtonResetCamera->setObjectName(QString::fromUtf8("pushButtonResetCamera"));
        pushButtonResetCamera->setMaximumSize(QSize(149, 16777215));

        horizontalLayout->addWidget(pushButtonResetCamera);


        verticalLayout->addLayout(horizontalLayout);


        verticalLayout_2->addLayout(verticalLayout);

        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        mainToolBar->addSeparator();
        mainToolBar->addAction(actionOpenFile);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionOpenFile->setText(QApplication::translate("MainWindow", "OpenFile", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionOpenFile->setToolTip(QApplication::translate("MainWindow", "Open File..", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBoxMeshSurface->setText(QApplication::translate("MainWindow", "Show Grid", 0, QApplication::UnicodeUTF8));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0, QApplication::UnicodeUTF8));
        pushButtonResetCamera->setText(QApplication::translate("MainWindow", "Reset Camera", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
