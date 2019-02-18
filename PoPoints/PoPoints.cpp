#include "PoPoints.h"

PoPoints::PoPoints(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//
	/***** Slots connection of QMenuBar and QBar *****/
	// File (connect)
	QObject::connect(ui.openAction, &QAction::triggered, this, &PoPoints::open);

	// Edit (connect)
	QObject::connect(ui.translateAction, &QAction::triggered, this, &PoPoints::transformLastCloud);
	QObject::connect(ui.addGaussNoiseAction, &QAction::triggered, this, &PoPoints::addGaussNoise);
	QObject::connect(ui.addOutlierAction, &QAction::triggered, this, &PoPoints::addOutlier);

	// Tool (connect)
	QObject::connect(ui.cubeAction, &QAction::triggered, this, &PoPoints::generateCube);
	QObject::connect(ui.cubeshellAction, &QAction::triggered, this, &PoPoints::generateCubeShell);
	QObject::connect(ui.sphereAction, &QAction::triggered, this, &PoPoints::generateSphere);
	QObject::connect(ui.sphereshellAction, &QAction::triggered, this, &PoPoints::generateSphereShell);

	// algorithm (connect)
	QObject::connect(ui.icpAction, &QAction::triggered, this, &PoPoints::icp);
	QObject::connect(ui.ransac_planeAction, &QAction::triggered, this, &PoPoints::ransac_plane);
	QObject::connect(ui.ransac_sphereAction, &QAction::triggered, this, &PoPoints::ransac_sphere);
	QObject::connect(ui.actionSfM, &QAction::triggered, this, &PoPoints::naive_sfm);





	/***** Slots connection of dataTree(QTreeWidget) widget *****/
// Item in dataTree is left-clicked (connect)
	QObject::connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(itemSelected(QTreeWidgetItem*, int)));
	// Item in dataTree is right-clicked
	QObject::connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(itemPopmenu(const QPoint&)));

	// About (connect)
	QObject::connect(ui.aboutAction, &QAction::triggered, this, &PoPoints::about);
	QObject::connect(ui.helpAction, &QAction::triggered, this, &PoPoints::help);

	//initialization

	QString qss = dark_qss;
	qApp->setStyleSheet(qss);

	// point cloud initialization
	cloud_now.reset(new PointCloudT);

	// visualization
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();

}

void PoPoints::open()
{
	QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(model_dirname.c_str()), tr("Point cloud data(*.pcd *.ply *.obj);;All file(*.*)"));
	//Return if filenames is empty
	if (filenames.isEmpty())
		return;

	// Clear cache

	viewer->removeAllPointClouds();

	PointCloudT::Ptr  cloud_temp;
	// Open point cloud one by one
	for (int i = 0; i != filenames.size(); i++) {
		// time start

		cloud_temp.reset(new PointCloudT); // Reset cloud
		QString filename = filenames[i];
		std::string file_name = filename.toStdString();
		std::string subname = getFileName(file_name);  //提取全路径中的文件名（带后缀）

		//更新状态栏
		ui.statusBar->showMessage(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(filenames.size()) + " point cloud loading...");

		int status = -1;
		if (filename.endsWith(".pcd", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPCDFile(file_name, *cloud_temp);
		}
		else if (filename.endsWith(".ply", Qt::CaseInsensitive))
		{
			status = pcl::io::loadPLYFile(file_name, *cloud_temp);
		}
		else if (filename.endsWith(".obj", Qt::CaseInsensitive))
		{
			status = pcl::io::loadOBJFile(file_name, *cloud_temp);
		}
		else
		{
			//提示：无法读取除了.ply .pcd .obj以外的文件
			QMessageBox::information(this, tr("File format error"),
				tr("Can't open files except .ply .pcd .obj"));
			return;
		}
		//提示：后缀没问题，但文件内容无法读取
		if (status != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}

		if (cloud_temp->points[0].r == 0 && cloud_temp->points[0].g == 0 && cloud_temp->points[0].b == 0)
		{
			setCloudRGB(cloud_temp, 255, 255, 255);
		}
		setCloudAlpha(cloud_temp, 255);

		// 最后导入的点云的信息

		cloud_show.push_back(cloud_temp);  //将点云导入点云容器
		total_points += cloud_temp->points.size();
	}
		
    // 输出窗口
	setConsole("Load Cloud(s)",  "We now have " + QString::number(total_points)+ " Points.");


	ui.statusBar->showMessage("Load Point Cloud Done.");
	updatePointcloud();

}



void PoPoints::transformLastCloud()
{

	if (cloud_show.size() < 1) {
		// to do dlg
		setConsole("transformLastCloud", " insufficient point cloud.");
		return;
	}
	PointCloudT::Ptr cloud_in(cloud_show.back());  // Original point cloud
	PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
	
	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)

	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4MatrixT(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud(*cloud_in, *cloud_tr, transformation_matrix);
	cloud_show.push_back(cloud_tr);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::addGaussNoise()
{
	//添加高斯噪声
	int size = cloud_now->size();
	if (size == 0) return;
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	*cloud_temp = *cloud_now;
	boost::mt19937 rng;
	rng.seed(static_cast<unsigned int>(time(0)));
	boost::normal_distribution<> nd(0, 10);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<>> var_nor(rng, nd);//boost 库随机数
	//添加噪声
	for (size_t point_i = 0; point_i < size; ++point_i)
	{
		if (!(point_i % 10)) continue;
		cloud_temp->points[point_i].x = cloud_now->points[point_i].x + static_cast<float> (var_nor());
		cloud_temp->points[point_i].y = cloud_now->points[point_i].y + static_cast<float> (var_nor());
		cloud_temp->points[point_i].z = cloud_now->points[point_i].z + static_cast<float> (var_nor());
	}
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();


}

void PoPoints::addOutlier()
{
	//添加高斯噪声
	int size = cloud_now->size();
	if (size == 0) return;
	std::cout << "Select : " << size << "point(s)" << std::endl;
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	*cloud_temp = *cloud_now;

	//添加噪声
	size_t i = 0;
	for (; i < size; ++i)
	{
		
		cloud_temp->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
	}

	*cloud_temp = *cloud_temp +  *cloud_now;
	std::cout << "After add Outlier : " << cloud_temp->size() << "point(s)" << std::endl;
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::generateCube()
{

	PointCloudT::Ptr cloud_temp(new PointCloudT);
	cloud_temp->width = 5000;         // 设置点云宽
	cloud_temp->height = 1;            // 设置点云高，高为1，说明为无组织点云
	cloud_temp->is_dense = false;
	cloud_temp->resize(cloud_temp->width * cloud_temp->height);     // 重置点云大小
	for (size_t i = 0; i != cloud_temp->size(); ++i)
	{
		cloud_temp->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::generateCubeShell()
{
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	cloud_temp->width = 6000;         // 设置点云宽
	cloud_temp->height = 1;            // 设置点云高，高为1，说明为无组织点云
	cloud_temp->is_dense = false;
	cloud_temp->resize(cloud_temp->width * cloud_temp->height);     // 重置点云大小
	size_t i = 0;

	for (; i != cloud_temp->size() / 3; ++i)
	{
		cloud_temp->points[i].x = (rand() / (RAND_MAX + 1.0f) > 0.5 ? 1024 : 0) - 512;
		cloud_temp->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}

	for (; i != cloud_temp->size() / 3 * 2; ++i)
	{
		cloud_temp->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].y = (rand() / (RAND_MAX + 1.0f) > 0.5 ? 1024 : 0) - 512;
		cloud_temp->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}

	for (; i != cloud_temp->size(); ++i)
	{
		cloud_temp->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f) - 512;
		cloud_temp->points[i].z = (rand() / (RAND_MAX + 1.0f) > 0.5 ? 1024 : 0) - 512;
		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::generateSphere()
{
	// populate our PointCloud with points
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	cloud_temp->width = 6000;
	cloud_temp->height = 1;
	cloud_temp->is_dense = false;
	cloud_temp->resize(cloud_temp->width * cloud_temp->height);
	float radius = 512;
	
	for (size_t i = 0; i < cloud_temp->points.size(); ++i)
	{
		int x, y, z;
		float dis;
		do
		{
			x = radius * 2 * rand() / (RAND_MAX + 1.0f) - radius;
			y = radius * 2 * rand() / (RAND_MAX + 1.0f) - radius;
			z = radius * 2 * rand() / (RAND_MAX + 1.0f) - radius;
			dis = x * x + y * y + z * z;

		} while (dis>radius*radius);



		cloud_temp->points[i].x = x;
		cloud_temp->points[i].y = y;
		cloud_temp->points[i].z = z;

		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::generateSphereShell()
{
	// populate our PointCloud with points
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	cloud_temp->width = 6000;        
	cloud_temp->height = 1;           
	cloud_temp->is_dense = false;
	cloud_temp->resize(cloud_temp->width * cloud_temp->height); 
	float radius = 512;
	for (size_t i = 0; i < cloud_temp->points.size(); ++i)
	{
		
		cloud_temp->points[i].x = radius * 2 * rand() / (RAND_MAX + 1.0f)- radius;
		cloud_temp->points[i].y = radius * 2 * rand() / (RAND_MAX + 1.0f)- radius;
		//cloud_temp->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
		if (i % 2 == 0)
			cloud_temp->points[i].z = sqrt(radius *radius - (cloud_temp->points[i].x * cloud_temp->points[i].x) - (cloud_temp->points[i].y * cloud_temp->points[i].y));
		else
			cloud_temp->points[i].z = -sqrt(radius* radius - (cloud_temp->points[i].x * cloud_temp->points[i].x) - (cloud_temp->points[i].y * cloud_temp->points[i].y));
		
		cloud_temp->points[i].r = 255;
		cloud_temp->points[i].g = 255;
		cloud_temp->points[i].b = 255;
	}
	cloud_show.push_back(cloud_temp);
	updatePointcloud();
	ui.qvtkWidget->update();

}

void PoPoints::pointcolorChanged()
{
	QColor color = QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

	if (color.isValid()) //判断所选的颜色是否有效
	{
		
		QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
		int selected_item_count = ui.dataTree->selectedItems().size();
		if (selected_item_count == 0) {
			for (int i = 0; i != cloud_show.size(); i++) {
				for (int j = 0; j != cloud_show[i]->points.size(); j++) {
					cloud_show[i]->points[j].r = color.red();
					cloud_show[i]->points[j].g = color.green();
					cloud_show[i]->points[j].b = color.blue();
				}
			}

		}
		else {
			for (int i = 0; i != selected_item_count; i++) {
				int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
				for (int j = 0; j != cloud_show[cloud_id]->size(); j++) {
					cloud_show[cloud_id]->points[j].r = color.red();
					cloud_show[cloud_id]->points[j].g = color.green();
					cloud_show[cloud_id]->points[j].b = color.blue();
				}
			}
			// 输出窗口
			setConsole("Change cloud color", "to " + QString::number(color.red()) + " " + QString::number(color.green()) + " " + QString::number(color.blue()));
		}
	}
	updatePointcloud();
}

void PoPoints::pointcolorRamdom()
{
	unsigned int r = 255 * (rand() / (RAND_MAX + 1.0f));
	unsigned int g = 255 * (rand() / (RAND_MAX + 1.0f));
	unsigned int b = 255 * (rand() / (RAND_MAX + 1.0f));
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			for (int j = 0; j != cloud_show[i]->points.size(); j++) {
				cloud_show[i]->points[j].r = r;
				cloud_show[i]->points[j].g = g;
				cloud_show[i]->points[j].b = b;
			}
		}

	}
	else {
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			for (int j = 0; j != cloud_show[cloud_id]->size(); j++) {
				cloud_show[cloud_id]->points[j].r = r;
				cloud_show[cloud_id]->points[j].g = g;
				cloud_show[cloud_id]->points[j].b = b;
			}
		}
		// 输出窗口
		setConsole("Change cloud color", "to random color." );
	}
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::pointHide()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			setCloudAlpha(cloud_show[i], 0);
		}

	}
	else {
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
				setCloudAlpha(cloud_show[cloud_id], 0);
		}
		// 输出窗口
		setConsole("Hide", "");
	}
	updatePointcloud();
	ui.qvtkWidget->update();

}

void PoPoints::pointShow()
{
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		for (int i = 0; i != cloud_show.size(); i++) {
			setCloudAlpha(cloud_show[i], 255);
		}

	}
	else {
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
			setCloudAlpha(cloud_show[cloud_id], 255);
		}
		// 输出窗口
		setConsole("Hide", "");
	}
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::icp()
{
	int size = cloud_show.size();
	if ( size < 2) {
	// to do dlg
		setConsole("icp", " insufficient point cloud.");
		return;
	}
	PointCloudT::Ptr cloud_in(cloud_show[size-2]);  // Original point cloud
	PointCloudT::Ptr cloud_out(cloud_show[size - 1]);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

	

  // The Iterative Closest Point algorithm
	pcl::console::TicToc time;
	int iterations = 10;  // Default number of ICP iterations
	time.tic();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	/*

	// Set the input source and target
	//icp.setInputCloud(cloud_source);
	//icp.setInputTarget(cloud_target);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance(0.05);
	// Set the maximum number of iterations (criterion 1)
	//icp.setMaximumIterations(50);
	// Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon(1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon(1);

	// Perform the alignment
	//icp.align(cloud_source_registered);
	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	//Eigen::Matrix4f transformation = icp.getFinalTransformation();
	*/
	
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	icp.align(*cloud_icp);// Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4MatrixT(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return ;
	}
	cloud_show.push_back(cloud_icp);
	updatePointcloud();
	ui.qvtkWidget->update();
}

void PoPoints::ransac_plane()
{
	// created RandomSampleConsensus object and compute the appropriated model
	pcl::PointCloud<PointT>::Ptr final(new PointCloudT);
	std::cout<<"cloud_now.size(): "<< cloud_now->size() << std::endl;
	pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud_now));
	
	pcl::RandomSampleConsensus<PointT> ransac(model_p);
	ransac.setDistanceThreshold(.01);
	ransac.computeModel();
	std::vector<int> inliers;
	ransac.getInliers(inliers);
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<PointT>(*cloud_now, inliers, *final);
	cloud_show.push_back(final);
	std::cout << "final.size(): " << final->size() << std::endl;
	updatePointcloud();

}

void PoPoints::ransac_sphere()
{
	// created RandomSampleConsensus object and compute the appropriated model
	pcl::PointCloud<PointT>::Ptr final(new PointCloudT); 
	pcl::SampleConsensusModelSphere<PointT>::Ptr model_s(new pcl::SampleConsensusModelSphere<PointT>(cloud_now));
	std::cout << "cloud_now.size(): " << cloud_now->size() << std::endl;
	pcl::RandomSampleConsensus<PointT> ransac(model_s);
	ransac.setDistanceThreshold(1);
	ransac.computeModel();
	std::vector<int> inliers;
	ransac.getInliers(inliers);
	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<PointT>(*cloud_now, inliers, *final);
	cloud_show.push_back(final);
	std::cout << "final.size(): " << final->size() << std::endl;
	updatePointcloud();
}

void PoPoints::naive_sfm()
{

	QStringList filenames = QFileDialog::getOpenFileNames(this, tr("Open point cloud file"), QString::fromLocal8Bit(model_dirname.c_str()), tr("All file(*.*)"));
	//Return if filenames is empty
	if (filenames.size()<2)
		return;

	// Clear cache

	viewer->removeAllPointClouds();

	PointCloudT::Ptr  cloud_temp;
	// Open picture cloud one by one
	for (int i = 0; i != filenames.size(); i++) {
		// time start

		cloud_temp.reset(new PointCloudT); // Reset cloud
		QString filename = filenames[i];
		std::string file_name = filename.toStdString();
		std::string subname = getFileName(file_name);  //提取全路径中的文件名（带后缀）

		//更新状态栏
		ui.statusBar->showMessage(QString::fromLocal8Bit(subname.c_str()) + ": " + QString::number(i) + "/" + QString::number(filenames.size()) + " point cloud loading...");

		int status = 0;


		// 读取源图像并转化为灰度图像
		cv::Mat srcImage = cv::imread(file_name);
		// 判断文件是否读入正确
		if (!srcImage.data)
			status = 1;
		else {
			// 图像显示
			//cv::imshow("srcImage", srcImage);
			// 等待键盘键入
			//cv::waitKey(0);
		}
		//提示：后缀没问题，但文件内容无法读取
		if (status != 0)
		{
			QMessageBox::critical(this, tr("Reading file error"), tr("We can not open the file"));
			return;
		}
		//
	}
		cv::Mat img_1 = cv::imread(filenames[0].toStdString());
		cv::Mat img_2 = cv::imread(filenames[1].toStdString());
		if (!img_1.data || !img_2.data)
		{
			cout << "error reading images " << endl;
			return;
		}
		auto orb = cv::ORB::create();
		//cv::ORB orb;
		std::vector<cv::KeyPoint> keyPoints_1, keyPoints_2;
		cv::Mat descriptors_1, descriptors_2;
		
		orb->detect(img_1, keyPoints_1);
		orb->detect(img_2, keyPoints_2);

		orb->compute(img_1, keyPoints_1, descriptors_1);
		orb->compute(img_2, keyPoints_2, descriptors_2);

		//std::vector<cv::DMatch> matches;
		std::vector<std::vector<cv::DMatch>> radius_matches;
		cv::BFMatcher bfMatcher(cv::NORM_HAMMING);
		//bfMatcher.match(descriptors_1, descriptors_2, matches);
		bfMatcher.radiusMatch(descriptors_1, descriptors_2, radius_matches, 20);
		double max_dist = 0; double min_dist = 100;
		std::vector< cv::DMatch > good_matches;
		std::cout << " radius_matches.size " << radius_matches.size() << endl;
		std::cout << " radius_matches.size " << radius_matches[0].size() << endl;

		for (size_t i = 0; i < radius_matches.size(); i++)
			if(radius_matches[i].size()>0) {
			const cv::DMatch& bestMatch = radius_matches[i][0];
			good_matches.push_back(bestMatch);
		}
		std::cout << " good_matches.size " << good_matches.size() << endl;

		cv::Mat outImage;
		cv::drawMatches(img_1, keyPoints_1, img_1, keyPoints_1, good_matches, outImage,
			cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		cv::namedWindow("out", cv::WINDOW_NORMAL);//CV_WINDOW_NORMAL就是0
		cv::resizeWindow("out", 640, 480);
		cv::imshow("out",outImage);

		//匹配了一些点，完成。



		cv::waitKey(0);




}


void PoPoints::about()
{
	AboutWin *aboutwin = new AboutWin(this);
	aboutwin->setModal(true);
	aboutwin->show();

	// 输出窗口
	setConsole("About", "All copyright given up.");

}

void PoPoints::help()
{
	HelpWin *helptwin = new HelpWin(this);
	helptwin->setModal(true);
	helptwin->show();

	// 输出窗口
	setConsole("Help", "Check the blog, will you?");
}

std::string PoPoints::getFileName(std::string file_name)
{

	std::string subname;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subname.insert(subname.begin(), *i);
	}
	return subname;
	
}

void PoPoints::print4x4MatrixT(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void PoPoints::keyboardEventOccurred(const pcl::visualization::KeyboardEvent & event, void * nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}


void PoPoints::setCloudRGB(PointCloudT::Ptr &cloud,unsigned int r, unsigned int g, unsigned int b)
{
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].r = r;
		cloud->points[i].g = g;
		cloud->points[i].b = b;
		cloud->points[i].a = 255;
	}
}

void PoPoints::setCloudAlpha(PointCloudT::Ptr & cloud, unsigned int a)
{
	for (size_t i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].a = a;
	}
}

void PoPoints::setConsole(QString operation, QString detail)
{

	int rows = ui.consoleTable->rowCount();
	ui.consoleTable->setRowCount(++rows);
	QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
	QString time_str = time.toString("MM-dd hh:mm:ss"); //设置显示格式
	ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(time_str));
	ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(detail));

	ui.consoleTable->scrollToBottom(); // 滑动自动滚到最底部
}

void PoPoints::itemSelected(QTreeWidgetItem * item, int count)
{
	count = ui.dataTree->indexOfTopLevelItem(item);  //获取item的行号

	for (int i = 0; i != cloud_show.size(); i++)
	{
		viewer->updatePointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + QString::number(i).toStdString());
	}

	*cloud_now = *cloud_show[count];


	//选中item所对应的点云尺寸变大
	QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	int selected_item_count = ui.dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++) {
		int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
			2, "cloud" + QString::number(cloud_id).toStdString());
	}

	ui.qvtkWidget->update();

	setConsole("itemSelected", QString::number(selected_item_count)+ " item(s) selected.");

}

void PoPoints::itemPopmenu(const QPoint &)
{
	QTreeWidgetItem* curItem = ui.dataTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL)return;           //这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
	QString name = curItem->text(0);
	int id = ui.dataTree->indexOfTopLevelItem(curItem);
	std::string cloud_id = "cloud" + QString::number(id).toStdString();


	//QAction deleteItemAction("Delete", this);
	QAction hideItemAction("Hide", this);
	QAction showItemAction("Show", this);
	QAction changeColorAction("Change color", this);
	QAction randomColorAction("Random color", this);


	//connect(&deleteItemAction, &QAction::triggered, this, &PoPoints::deleteItem);
	connect(&hideItemAction, &QAction::triggered, this, &PoPoints::pointHide);
	connect(&showItemAction, &QAction::triggered, this, &PoPoints::pointShow);
	connect(&changeColorAction, &QAction::triggered, this, &PoPoints::pointcolorChanged);
	connect(&randomColorAction, &QAction::triggered, this, &PoPoints::pointcolorRamdom);

	//QPoint pos;
	QMenu menu(ui.dataTree);
	menu.addAction(&hideItemAction);
	menu.addAction(&showItemAction);
	//menu.addAction(&deleteItemAction);
	menu.addAction(&changeColorAction);
	menu.addAction(&randomColorAction);

	//if (mycloud_vec[id].visible == true) {
	//	menu.actions()[1]->setVisible(false);
	//	menu.actions()[0]->setVisible(true);
	//}
	//else {
	//	menu.actions()[1]->setVisible(true);
	//	menu.actions()[0]->setVisible(false);
	//}


	menu.exec(QCursor::pos()); //在当前鼠标位置显示

	setConsole("popMenu", "popMenu");
}

void PoPoints::updatePointcloud()
{
	viewer->removeAllPointClouds();
	for (int i = 0; i != cloud_show.size(); i++)
	{
		viewer->addPointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
		viewer->updatePointCloud(cloud_show[i], "cloud" + QString::number(i).toStdString());
	}
	//viewer->resetCamera();
	//ui.screen->update();
	updateDataTree();
	

}

void PoPoints::updateConsoleTable()
{
}

void PoPoints::updateDataTree()
{
	ui.dataTree->clear();
	//更新资源管理树
	for (int i = 0; i != cloud_show.size(); i++)
	{
		QTreeWidgetItem *cloudName = new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit(std::to_string(i).c_str()));
		cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
		ui.dataTree->addTopLevelItem(cloudName);
	}


}

void PoPoints::updatetPropertyTable()
{
}
