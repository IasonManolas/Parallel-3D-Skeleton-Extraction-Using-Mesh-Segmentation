/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
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
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolBox>
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
    QHBoxLayout *horizontalLayout;
    GLWidget *openGLWidget;
    QVBoxLayout *verticalLayout;
    QToolBox *toolBox;
    QWidget *sceneRelated;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer_2;
    QCheckBox *checkBoxShowVertices;
    QCheckBox *checkBoxMeshSurface;
    QCheckBox *checkBoxAxis;
    QPushButton *pushButtonResetCamera;
    QWidget *contractionRelated;
    QVBoxLayout *verticalLayout_3;
    QSpacerItem *verticalSpacer;
    QCheckBox *laplacianHeatMapCheckBox;
    QCheckBox *contractionModeCheckBox;
    QSpinBox *contractionThresholdSpinBox;
    QWidget *skeletonPage;
    QVBoxLayout *verticalLayout_4;
    QSpacerItem *verticalSpacer_3;
    QPushButton *clearSkeletonButton;
    QToolBar *mainToolBar;
    QMenuBar *menuBar;
    QMenu *menuSave;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(760, 499);
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
        horizontalLayout = new QHBoxLayout(centralWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        openGLWidget = new GLWidget(centralWidget);
        openGLWidget->setObjectName(QStringLiteral("openGLWidget"));
        openGLWidget->setMinimumSize(QSize(600, 0));

        horizontalLayout->addWidget(openGLWidget);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        toolBox = new QToolBox(centralWidget);
        toolBox->setObjectName(QStringLiteral("toolBox"));
        toolBox->setFrameShape(QFrame::NoFrame);
        toolBox->setFrameShadow(QFrame::Plain);
        sceneRelated = new QWidget();
        sceneRelated->setObjectName(QStringLiteral("sceneRelated"));
        sceneRelated->setGeometry(QRect(0, 0, 134, 337));
        verticalLayout_2 = new QVBoxLayout(sceneRelated);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        checkBoxShowVertices = new QCheckBox(sceneRelated);
        checkBoxShowVertices->setObjectName(QStringLiteral("checkBoxShowVertices"));

        verticalLayout_2->addWidget(checkBoxShowVertices);

        checkBoxMeshSurface = new QCheckBox(sceneRelated);
        checkBoxMeshSurface->setObjectName(QStringLiteral("checkBoxMeshSurface"));
        checkBoxMeshSurface->setEnabled(true);
        checkBoxMeshSurface->setChecked(false);
        checkBoxMeshSurface->setTristate(true);

        verticalLayout_2->addWidget(checkBoxMeshSurface);

        checkBoxAxis = new QCheckBox(sceneRelated);
        checkBoxAxis->setObjectName(QStringLiteral("checkBoxAxis"));

        verticalLayout_2->addWidget(checkBoxAxis);

        pushButtonResetCamera = new QPushButton(sceneRelated);
        pushButtonResetCamera->setObjectName(QStringLiteral("pushButtonResetCamera"));
        pushButtonResetCamera->setMaximumSize(QSize(149, 16777215));

        verticalLayout_2->addWidget(pushButtonResetCamera);

        toolBox->addItem(sceneRelated, QStringLiteral("Scene"));
        contractionRelated = new QWidget();
        contractionRelated->setObjectName(QStringLiteral("contractionRelated"));
        contractionRelated->setGeometry(QRect(0, 0, 137, 323));
        verticalLayout_3 = new QVBoxLayout(contractionRelated);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        laplacianHeatMapCheckBox = new QCheckBox(contractionRelated);
        laplacianHeatMapCheckBox->setObjectName(QStringLiteral("laplacianHeatMapCheckBox"));

        verticalLayout_3->addWidget(laplacianHeatMapCheckBox);

        contractionModeCheckBox = new QCheckBox(contractionRelated);
        contractionModeCheckBox->setObjectName(QStringLiteral("contractionModeCheckBox"));
        contractionModeCheckBox->setChecked(true);

        verticalLayout_3->addWidget(contractionModeCheckBox);

        contractionThresholdSpinBox = new QSpinBox(contractionRelated);
        contractionThresholdSpinBox->setObjectName(QStringLiteral("contractionThresholdSpinBox"));
        contractionThresholdSpinBox->setMinimum(-8);
        contractionThresholdSpinBox->setMaximum(-1);
        contractionThresholdSpinBox->setValue(-4);

        verticalLayout_3->addWidget(contractionThresholdSpinBox);

        toolBox->addItem(contractionRelated, QStringLiteral("Contraction"));
        skeletonPage = new QWidget();
        skeletonPage->setObjectName(QStringLiteral("skeletonPage"));
        skeletonPage->setGeometry(QRect(0, 0, 134, 337));
        verticalLayout_4 = new QVBoxLayout(skeletonPage);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_3);

        clearSkeletonButton = new QPushButton(skeletonPage);
        clearSkeletonButton->setObjectName(QStringLiteral("clearSkeletonButton"));

        verticalLayout_4->addWidget(clearSkeletonButton);

        toolBox->addItem(skeletonPage, QStringLiteral("Skeleton"));

        verticalLayout->addWidget(toolBox);


        horizontalLayout->addLayout(verticalLayout);

        horizontalLayout->setStretch(0, 80);
        horizontalLayout->setStretch(1, 20);
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 760, 19));
        menuSave = new QMenu(menuBar);
        menuSave->setObjectName(QStringLiteral("menuSave"));
        MainWindow->setMenuBar(menuBar);

        mainToolBar->addSeparator();
        mainToolBar->addAction(actionOpenFile);
        menuBar->addAction(menuSave->menuAction());
        menuSave->addAction(actionSave_Model);
        menuSave->addAction(actionSave_Segment);

        retranslateUi(MainWindow);

        toolBox->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Thesis", 0));
        actionOpenFile->setText(QApplication::translate("MainWindow", "OpenFile", 0));
#ifndef QT_NO_TOOLTIP
        actionOpenFile->setToolTip(QApplication::translate("MainWindow", "Open File..", 0));
#endif // QT_NO_TOOLTIP
        actionSave_Model->setText(QApplication::translate("MainWindow", "Save Model", 0));
        actionSave_Segment->setText(QApplication::translate("MainWindow", "Save Segment", 0));
        checkBoxShowVertices->setText(QApplication::translate("MainWindow", "Vertices", 0));
        checkBoxMeshSurface->setText(QApplication::translate("MainWindow", "Grid", 0));
        checkBoxAxis->setText(QApplication::translate("MainWindow", "Axis", 0));
        pushButtonResetCamera->setText(QApplication::translate("MainWindow", "Reset", 0));
        toolBox->setItemText(toolBox->indexOf(sceneRelated), QApplication::translate("MainWindow", "Scene", 0));
        laplacianHeatMapCheckBox->setText(QApplication::translate("MainWindow", "Show Laplacian \n"
"      Heat Map", 0));
        contractionModeCheckBox->setText(QApplication::translate("MainWindow", "  Automatic \n"
" Contraction", 0));
        toolBox->setItemText(toolBox->indexOf(contractionRelated), QApplication::translate("MainWindow", "Contraction", 0));
        clearSkeletonButton->setText(QApplication::translate("MainWindow", "Clear Skeleton", 0));
        toolBox->setItemText(toolBox->indexOf(skeletonPage), QApplication::translate("MainWindow", "Skeleton", 0));
        menuSave->setTitle(QApplication::translate("MainWindow", "Save", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
