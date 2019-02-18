#pragma once

// std
#include <vector>
#include <string>
#include <algorithm>

// Qt
#include <QtWidgets/QMainWindow>
#include <QString>
#include <QDebug>
#include <QLabel>
#include <QMessageBox>
#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QFileDialog>
#include <QColorDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <QTextEdit>
#include <QTime>
#include <QMouseEvent> 
#include <QDesktopServices> 
#include <QUrl>
#include "ui_PoPoints.h"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/registration/icp.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Custom Win
#include "AboutWin.h"
#include "HelpWin.h"


//opencv

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



//static
static QString dark_qss = "QWidget{ 	 	background-color: rgb(60, 63, 65); }  QDockWidget{ 	color: rgb(208, 208, 208);	 	background-color: rgb(60, 63, 65);  	border-color: rgb(63, 63, 70); 	border-top-color: rgb(255, 255, 255);	 	font: 10pt \"Microsoft YaHei UI\"; }  QTableWidget{	 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }  QTreeWidget{ 	background-color: rgb(43, 43, 43);	 	border-color: rgb(63, 63, 70); 	color: rgb(241, 241, 241); 	alternate-background-color: rgb(85, 85, 85); 	font: 9pt \"Microsoft YaHei UI\"; }   QHeaderView::section{ 	background-color: rgb(53, 53, 53); 	color: rgb(241, 241, 241); 	border:0px solid #E0DDDC; 	border-bottom:1px solid #262626; 	height: 30px; }   QToolBar{ 	background-color: rgb(60, 63, 65); 	border-bottom: 1px solid #262626; }  QStatusBar{ 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; }  QMenuBar{ 	background-color: rgb(60, 63, 65); 	color: rgb(241, 241, 241); 	font: 9pt \"Microsoft YaHei UI\"; 	border-bottom: 1px solid #262626; }  QMenuBar::item:selected{ 	background-color: rgb(75, 110, 175); }  QMenu{ 	font: 9pt \"Microsoft YaHei UI\";	 	color: rgb(241, 241, 241); 	background-color: rgb(60, 63, 65); }  QMenu::item:selected{ 	background-color: rgb(75, 110, 175); }   QLabel{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QCheckBox{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QLCDNumber{ 	color: rgb(241, 241, 241); 	font: 10pt \"Microsoft YaHei UI\"; }  QPushButton{ 	color: rgb(241, 241, 241); 	 	background-color: rgb(73, 78, 80); } ";


class PoPoints : public QMainWindow
{
	Q_OBJECT

public:
	PoPoints(QWidget *parent = Q_NULLPTR);

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PointCloudT::Ptr cloud_now;
	std::vector< PointCloudT::Ptr> cloud_show;

	// File menu slots
	void open();
	//void add();
	//void saveWholeScaninTree();
	//void clear();
	//void save();
	//void saveBinary();
	//void savemulti();
	//void change();
	//void exit();

	// Edit menu slots
	void transformLastCloud();
	void addGaussNoise();
	void addOutlier();

	// Tool slots
	void generateCube();
	void generateCubeShell();
	void generateSphere();
	void generateSphereShell();

	// Display menu slots
	void pointcolorChanged();
	void pointcolorRamdom();
	void pointHide();
	void pointShow();

	//void bgcolorChanged();


	//// View menu slots
	//void data();
	//void properties();
	//void console();
	//void rgbDock();
	//// Generate menu slots
	//void cube();

	// algorithm menu slots
	// ICP
	void icp();
	void ransac_plane();
	void ransac_sphere();
	void naive_sfm();


	// About
	void about();
	void help();

	// info initialization
	std::string model_dirname = "D:\\everglow\\PoPoints\\PoPoints\\models";
	long total_points = 0; //Total amount of points in the viewer



public:
	// Utils
	std::string getFileName(std::string file_name);
	void print4x4MatrixT(const Eigen::Matrix4d & matrix);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing);


	
	// UI Related Utils
	void setCloudRGB(PointCloudT::Ptr &cloud, unsigned int r, unsigned int g, unsigned int b);
	void setCloudAlpha(PointCloudT::Ptr &cloud, unsigned int a);
	void setConsole(QString operation, QString detail);



	void updatePointcloud();
	void updatetPropertyTable();
	void updateConsoleTable();
	void updateDataTree();



public slots:
	void itemSelected(QTreeWidgetItem* item, int count);
	void itemPopmenu(const QPoint&);


private:
	// algorithm variables
	
	bool next_iteration = false;
	

private:
	Ui::PoPointsClass ui;
};
