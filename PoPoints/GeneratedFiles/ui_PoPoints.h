/********************************************************************************
** Form generated from reading UI file 'PoPoints.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POPOINTS_H
#define UI_POPOINTS_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PoPointsClass
{
public:
    QAction *openAction;
    QAction *icpAction;
    QAction *rotateAction;
    QAction *translateAction;
    QAction *ransac_planeAction;
    QAction *ransac_sphereAction;
    QAction *aboutAction;
    QAction *helpAction;
    QAction *cubeAction;
    QAction *sphereAction;
    QAction *cubeshellAction;
    QAction *sphereshellAction;
    QAction *addGaussNoiseAction;
    QAction *addOutlierAction;
    QAction *actionSfM;
    QWidget *centralWidget;
    QDockWidget *dataDock;
    QWidget *dockWidgetContents_5;
    QVBoxLayout *verticalLayout_4;
    QTreeWidget *dataTree;
    QVTKWidget *qvtkWidget;
    QDockWidget *consoleDock;
    QWidget *dockWidgetContents_7;
    QVBoxLayout *verticalLayout_3;
    QTableWidget *consoleTable;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuEdit;
    QMenu *menuTool;
    QMenu *menuGenerate;
    QMenu *menuAbout;
    QMenu *menuDemo;
    QMenu *menuRANSAC;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PoPointsClass)
    {
        if (PoPointsClass->objectName().isEmpty())
            PoPointsClass->setObjectName(QStringLiteral("PoPointsClass"));
        PoPointsClass->resize(1280, 720);
        openAction = new QAction(PoPointsClass);
        openAction->setObjectName(QStringLiteral("openAction"));
        icpAction = new QAction(PoPointsClass);
        icpAction->setObjectName(QStringLiteral("icpAction"));
        rotateAction = new QAction(PoPointsClass);
        rotateAction->setObjectName(QStringLiteral("rotateAction"));
        translateAction = new QAction(PoPointsClass);
        translateAction->setObjectName(QStringLiteral("translateAction"));
        ransac_planeAction = new QAction(PoPointsClass);
        ransac_planeAction->setObjectName(QStringLiteral("ransac_planeAction"));
        ransac_sphereAction = new QAction(PoPointsClass);
        ransac_sphereAction->setObjectName(QStringLiteral("ransac_sphereAction"));
        aboutAction = new QAction(PoPointsClass);
        aboutAction->setObjectName(QStringLiteral("aboutAction"));
        helpAction = new QAction(PoPointsClass);
        helpAction->setObjectName(QStringLiteral("helpAction"));
        cubeAction = new QAction(PoPointsClass);
        cubeAction->setObjectName(QStringLiteral("cubeAction"));
        sphereAction = new QAction(PoPointsClass);
        sphereAction->setObjectName(QStringLiteral("sphereAction"));
        cubeshellAction = new QAction(PoPointsClass);
        cubeshellAction->setObjectName(QStringLiteral("cubeshellAction"));
        sphereshellAction = new QAction(PoPointsClass);
        sphereshellAction->setObjectName(QStringLiteral("sphereshellAction"));
        addGaussNoiseAction = new QAction(PoPointsClass);
        addGaussNoiseAction->setObjectName(QStringLiteral("addGaussNoiseAction"));
        addOutlierAction = new QAction(PoPointsClass);
        addOutlierAction->setObjectName(QStringLiteral("addOutlierAction"));
        actionSfM = new QAction(PoPointsClass);
        actionSfM->setObjectName(QStringLiteral("actionSfM"));
        centralWidget = new QWidget(PoPointsClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        dataDock = new QDockWidget(centralWidget);
        dataDock->setObjectName(QStringLiteral("dataDock"));
        dataDock->setGeometry(QRect(1060, 0, 220, 600));
        QFont font;
        font.setFamily(QString::fromUtf8("\345\276\256\350\275\257\351\233\205\351\273\221"));
        font.setPointSize(10);
        dataDock->setFont(font);
        dockWidgetContents_5 = new QWidget();
        dockWidgetContents_5->setObjectName(QStringLiteral("dockWidgetContents_5"));
        verticalLayout_4 = new QVBoxLayout(dockWidgetContents_5);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        dataTree = new QTreeWidget(dockWidgetContents_5);
        dataTree->setObjectName(QStringLiteral("dataTree"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\345\276\256\350\275\257\351\233\205\351\273\221"));
        font1.setPointSize(9);
        dataTree->setFont(font1);
        dataTree->setContextMenuPolicy(Qt::CustomContextMenu);

        verticalLayout_4->addWidget(dataTree);

        dataDock->setWidget(dockWidgetContents_5);
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(10, 10, 1040, 580));
        consoleDock = new QDockWidget(centralWidget);
        consoleDock->setObjectName(QStringLiteral("consoleDock"));
        consoleDock->setGeometry(QRect(0, 600, 1280, 132));
        dockWidgetContents_7 = new QWidget();
        dockWidgetContents_7->setObjectName(QStringLiteral("dockWidgetContents_7"));
        verticalLayout_3 = new QVBoxLayout(dockWidgetContents_7);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        consoleTable = new QTableWidget(dockWidgetContents_7);
        if (consoleTable->columnCount() < 3)
            consoleTable->setColumnCount(3);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setTextAlignment(Qt::AlignLeading|Qt::AlignVCenter);
        consoleTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        __qtablewidgetitem1->setTextAlignment(Qt::AlignLeading|Qt::AlignVCenter);
        consoleTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        __qtablewidgetitem2->setTextAlignment(Qt::AlignLeading|Qt::AlignVCenter);
        consoleTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        consoleTable->setObjectName(QStringLiteral("consoleTable"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(consoleTable->sizePolicy().hasHeightForWidth());
        consoleTable->setSizePolicy(sizePolicy);
        consoleTable->setMinimumSize(QSize(0, 100));
        consoleTable->setShowGrid(false);
        consoleTable->setGridStyle(Qt::SolidLine);
        consoleTable->setRowCount(0);
        consoleTable->setColumnCount(3);
        consoleTable->horizontalHeader()->setVisible(true);
        consoleTable->horizontalHeader()->setDefaultSectionSize(200);
        consoleTable->horizontalHeader()->setStretchLastSection(true);
        consoleTable->verticalHeader()->setVisible(false);

        verticalLayout_3->addWidget(consoleTable);

        consoleDock->setWidget(dockWidgetContents_7);
        PoPointsClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PoPointsClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1280, 19));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuEdit = new QMenu(menuBar);
        menuEdit->setObjectName(QStringLiteral("menuEdit"));
        menuTool = new QMenu(menuBar);
        menuTool->setObjectName(QStringLiteral("menuTool"));
        menuGenerate = new QMenu(menuTool);
        menuGenerate->setObjectName(QStringLiteral("menuGenerate"));
        menuAbout = new QMenu(menuBar);
        menuAbout->setObjectName(QStringLiteral("menuAbout"));
        menuDemo = new QMenu(menuBar);
        menuDemo->setObjectName(QStringLiteral("menuDemo"));
        menuRANSAC = new QMenu(menuDemo);
        menuRANSAC->setObjectName(QStringLiteral("menuRANSAC"));
        PoPointsClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PoPointsClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        PoPointsClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PoPointsClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        PoPointsClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuEdit->menuAction());
        menuBar->addAction(menuTool->menuAction());
        menuBar->addAction(menuDemo->menuAction());
        menuBar->addAction(menuAbout->menuAction());
        menuFile->addAction(openAction);
        menuEdit->addAction(translateAction);
        menuEdit->addAction(addGaussNoiseAction);
        menuEdit->addAction(addOutlierAction);
        menuTool->addAction(menuGenerate->menuAction());
        menuGenerate->addAction(cubeAction);
        menuGenerate->addAction(cubeshellAction);
        menuGenerate->addAction(sphereAction);
        menuGenerate->addAction(sphereshellAction);
        menuAbout->addAction(aboutAction);
        menuAbout->addAction(helpAction);
        menuDemo->addAction(icpAction);
        menuDemo->addAction(menuRANSAC->menuAction());
        menuDemo->addAction(actionSfM);
        menuRANSAC->addAction(ransac_planeAction);
        menuRANSAC->addAction(ransac_sphereAction);

        retranslateUi(PoPointsClass);

        QMetaObject::connectSlotsByName(PoPointsClass);
    } // setupUi

    void retranslateUi(QMainWindow *PoPointsClass)
    {
        PoPointsClass->setWindowTitle(QApplication::translate("PoPointsClass", "PoPoints", nullptr));
        openAction->setText(QApplication::translate("PoPointsClass", "open", nullptr));
        icpAction->setText(QApplication::translate("PoPointsClass", "ICP", nullptr));
        rotateAction->setText(QApplication::translate("PoPointsClass", "Rotate", nullptr));
        translateAction->setText(QApplication::translate("PoPointsClass", "Transform", nullptr));
        ransac_planeAction->setText(QApplication::translate("PoPointsClass", "Plane", nullptr));
        ransac_sphereAction->setText(QApplication::translate("PoPointsClass", "Sphere", nullptr));
        aboutAction->setText(QApplication::translate("PoPointsClass", "About", nullptr));
        helpAction->setText(QApplication::translate("PoPointsClass", "Help", nullptr));
        cubeAction->setText(QApplication::translate("PoPointsClass", "Cube", nullptr));
        sphereAction->setText(QApplication::translate("PoPointsClass", "Sphere", nullptr));
        cubeshellAction->setText(QApplication::translate("PoPointsClass", "CubeShell", nullptr));
        sphereshellAction->setText(QApplication::translate("PoPointsClass", "SphereShell", nullptr));
        addGaussNoiseAction->setText(QApplication::translate("PoPointsClass", "Add Noise", nullptr));
        addOutlierAction->setText(QApplication::translate("PoPointsClass", "Add Outlier", nullptr));
        actionSfM->setText(QApplication::translate("PoPointsClass", "SfM", nullptr));
        dataDock->setWindowTitle(QApplication::translate("PoPointsClass", "PointCloud", nullptr));
        QTreeWidgetItem *___qtreewidgetitem = dataTree->headerItem();
        ___qtreewidgetitem->setText(0, QApplication::translate("PoPointsClass", "Point Cloud Data", nullptr));
        consoleDock->setWindowTitle(QApplication::translate("PoPointsClass", "Console", nullptr));
        QTableWidgetItem *___qtablewidgetitem = consoleTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("PoPointsClass", "Time", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = consoleTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("PoPointsClass", "Event", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = consoleTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("PoPointsClass", "Detail", nullptr));
        menuFile->setTitle(QApplication::translate("PoPointsClass", "File", nullptr));
        menuEdit->setTitle(QApplication::translate("PoPointsClass", "Edit", nullptr));
        menuTool->setTitle(QApplication::translate("PoPointsClass", "Tool", nullptr));
        menuGenerate->setTitle(QApplication::translate("PoPointsClass", "Generate", nullptr));
        menuAbout->setTitle(QApplication::translate("PoPointsClass", "About", nullptr));
        menuDemo->setTitle(QApplication::translate("PoPointsClass", "Algorithm", nullptr));
        menuRANSAC->setTitle(QApplication::translate("PoPointsClass", "RANSAC", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PoPointsClass: public Ui_PoPointsClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POPOINTS_H
