#include "stdafx.h"
#include "PCLManager.h"

namespace pcl_func {

	//pcl::visualization::CloudViewer clusterViewer("Master: Clusters");
	//pcl::visualization::CloudViewer cHullViewer("Master: Concave Hull");
	//pcl::visualization::CloudViewer vlpViewer("VLP");

	PCLManager::PCLManager()
	{
		tracker.reset(new pcl::tracking::PyramidalKLTTracker<PointType>);
	}

	void PCLManager::update(Sensor &sensor, pcl::PointCloud<PointType>::Ptr &outputPoints)
	{
		//Receive PointCloud
		pcl::PointCloud<PointType>::Ptr calcPoints(new pcl::PointCloud<PointType>());
		*calcPoints = *outputPoints;

		//Filter
		//float voxelVal = 0.01f;
		//voxelGridFilter(voxelVal, calcPoints);
		//statisticalOutlierFilter(calcPoints);

		transformToZeroPoint(calcPoints, sensor, calcPoints);
		passThroughFilter(calcPoints, "x", sensor.getMinX(), sensor.getMaxX());
		passThroughFilter(calcPoints, "y", sensor.getMinY(), sensor.getMaxY());
		passThroughFilter(calcPoints, "z", sensor.getMinZ(), sensor.getMaxZ());

		// Visualizer
		visualizer.updateVisualizer(calcPoints);
		

		//-----------------------------------
		//クラスタ分割
		//-----------------------------------
		//ポイントクラウドをクラスタに分割
		//voxelGridFilter(0.1f, calcPoints);
		euclideanClusterExtraction(calcPoints, eachClouds);
		calcPoints.reset();

		//重心計算
		centroids.clear();
		for (int i = 0; i < eachClouds.size(); i++)
		{
			//クラスタの重心を計算
			Eigen::Vector4f center = centroid(eachClouds[i]);
			centroids.push_back(center);
		}

		//分割されたクラスタに色を付けて表示
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredColorCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> coloredClouds;

		for (int i = 0; i < eachClouds.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::copyPointCloud(*eachClouds[i], *cc);
			coloredClouds.push_back(cc);

			double red = 255;
			double green = 255;
			double blue = 255;

			switch (i % 3)
			{
			case 0:
				red = 255;
				green = 0;
				blue = 0;
				break;
			case 1:
				red = 0;
				green = 255;
				blue = 0;
				break;
			case 2:
				red = 127;
				green = 0;
				blue = 255;
				break;
			default:
				break;
			}

#pragma omp parallel for
			for (int j = 0; j < coloredClouds[i]->points.size(); j++)
			{
				coloredClouds[i]->points[j].r = red;
				coloredClouds[i]->points[j].g = green;
				coloredClouds[i]->points[j].b = blue;
			}
			*clusteredColorCloud += *coloredClouds[i];
		}
		cout << "Cluster: " << eachClouds.size() << endl;
		//vlpViewer.showCloud(clusteredColorCloud);

		std::cout << "----------------------------------\n" << std::endl;
	}

	void PCLManager::registrationAll(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputClouds, Eigen::Matrix4f &matrix) {
		if (inputCloud->empty())
		{
			return;
		}

		pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*inputCloud, *p, matrix);
		*inputCloud = *p;
		p.reset();
		*outputClouds += *inputCloud;
	}

	void PCLManager::createPolygon(vector<pcl::PointCloud<PointType>::Ptr> &clouds) {
		//各キネクトのデプスに対してポリゴン生成
		polygonMeshs.clear();
		for (int i = 0; i < clouds.size(); i++)
		{
			voxelGridFilter(0.005f, clouds[i]);
			if (clouds[i]->size() > 100)
			{
				polygonMeshs.push_back(createMeshWithOFM(clouds[i]));;
			}
		}
		cout << "NUM Polygon Mesh: " << polygonMeshs.size() << endl;
	}

	void PCLManager::transform(pcl::PointCloud<PointType>::Ptr cloud, float x, float y, float z, float  pitch, float yaw, float roll)
	{
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		float pitchRad = pitch / 180.0 * M_PI;
		float yawRad = yaw / 180.0 * M_PI;
		float rollRad = roll / 180.0 * M_PI;
		//回転
		transform.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));
		transform.rotate(Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitY()));
		transform.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitZ()));

		//移動
		transform.translation() << x, y, z;

		//実行
		pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloud, *p, transform);
		*cloud = *p;
		p.reset();
	}

	void PCLManager::transformToZeroPoint(pcl::PointCloud<PointType>::Ptr inputCloud, Sensor& posture, pcl::PointCloud<PointType>::Ptr outputCloud)
	{
		pcl::PointCloud<PointType>::Ptr calcPoints(new pcl::PointCloud<PointType>());

		//RotateX
		float pitchRad = posture.getPitch() / 180.0 * M_PI;
		Eigen::Affine3f transformRotateX = Eigen::Affine3f::Identity();
		transformRotateX.rotate(Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*inputCloud, *outputCloud, transformRotateX);

		//RotateY
		float yawRad = posture.getYaw() / 180.0 * M_PI;
		Eigen::Affine3f transformRotateY = Eigen::Affine3f::Identity();
		transformRotateY.rotate(Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitY()));
		pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateY);

		//RotateZ
		float rollRad = posture.getRoll() / 180.0 * M_PI;
		Eigen::Affine3f transformRotateZ = Eigen::Affine3f::Identity();
		transformRotateZ.rotate(Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*outputCloud, *outputCloud, transformRotateZ);

		//移動
		Eigen::Affine3f transformMove = Eigen::Affine3f::Identity();
		transformMove.translation() << posture.getX(), posture.getY(), posture.getZ();
		pcl::transformPointCloud(*outputCloud, *outputCloud, transformMove);
	}

	void PCLManager::edgeRmoveFilter(pcl::PointCloud<PointType>::Ptr cloud) {
		//法線
		pcl::PointCloud<PointNormalType>::Ptr normal = createNormals(cloud);

		//法線の傾きが一定以上だったらスルー
		vector<int> removeIndex;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			if (abs(normal->points[i].normal_x / normal->points[i].normal_z) > 0.1)
			{
				removeIndex.push_back(i);
			}
		}

		for (int i = 0; i < removeIndex.size(); i++)
		{
			cloud->erase(cloud->points.begin() + removeIndex[i] - i);
		}
	}


	void PCLManager::statisticalOutlierFilter(pcl::PointCloud<PointType>::Ptr cloud)
	{
		pcl::StatisticalOutlierRemoval<PointType> sor;
		sor.setInputCloud(cloud);
		sor.setMeanK(10);				//近接何ポイントを使うか
		sor.setStddevMulThresh(1.0);	//この標準偏差以上をフィルターして切る

		pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
		sor.filter(*cloud_filtered);

		pcl::copyPointCloud(*cloud_filtered, *cloud);
		cloud_filtered.reset();
	}

	void PCLManager::voxelGridFilter(float leaf, pcl::PointCloud<PointType>::Ptr cloud)
	{
		//VoxelGrid
		pcl::VoxelGrid<PointType> grid;
		grid.setLeafSize(leaf, leaf, leaf);		//フィルター範囲設定
		grid.setInputCloud(cloud);
		pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>());
		grid.filter(*cloud_filtered);
		pcl::copyPointCloud(*cloud_filtered, *cloud);
		cloud_filtered.reset();
	}

	void PCLManager::extractIndices(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointIndices::Ptr inliners)
	{
		pcl::PointCloud<PointType>::Ptr tmp(new pcl::PointCloud<PointType>());
		pcl::copyPointCloud(*cloud, *tmp);

		pcl::ExtractIndices<PointType> extract;
		extract.setInputCloud(tmp);
		extract.setIndices(inliners);

		extract.setNegative(true);
		extract.filter(*cloud);
	}

	void PCLManager::radiusOutlinerFilter(pcl::PointCloud<PointType>::Ptr cloud) {
		pcl::RadiusOutlierRemoval<PointType> ror;
		ror.setInputCloud(cloud);
		ror.setRadiusSearch(0.05);
		ror.setMinNeighborsInRadius(2);
		pcl::PointCloud<PointType>::Ptr filterdCloud(new pcl::PointCloud<PointType>());
		ror.filter(*filterdCloud);
		*cloud = *filterdCloud;
		filterdCloud.reset();
	}

	void PCLManager::passThroughFilter(pcl::PointCloud<PointType>::Ptr inputCloud, const string &fieldName, float min, float max) {
		pcl::PassThrough<PointType> pass;
		pass.setInputCloud(inputCloud);
		pass.setFilterFieldName(fieldName);
		pass.setFilterLimits(min, max);

		pcl::PointCloud<PointType>::Ptr filterdCloud(new pcl::PointCloud<PointType>());
		pass.filter(*filterdCloud);
		*inputCloud = *filterdCloud;
	}

	void PCLManager::nanRemovalFilter(pcl::PointCloud<PointType>::Ptr cloud) {
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}

	pcl::PointIndices::Ptr PCLManager::getPlaneIndices(pcl::PointCloud<PointType>::Ptr cloud)
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliners(new pcl::PointIndices);

		pcl::SACSegmentation<PointType> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.03);

		seg.setInputCloud(cloud);
		seg.segment(*inliners, *coefficients);

		return inliners;
	}

	pcl::PointCloud<pcl::Normal>::Ptr PCLManager::createNormals(pcl::PointCloud<PointType>::Ptr cloud, int KSearh)
	{
		// Normal estimation
		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(KSearh);
		n.compute(*normals);

		//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

		return normals;
	}

	pcl::PointCloud<PointNormalType>::Ptr PCLManager::createNormals(pcl::PointCloud<PointType>::Ptr cloud)
	{
		//法線の取得
		//法線格納用のスマートポインタを用意
		pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
		//スムージング用のモジュールを用意
		pcl::MovingLeastSquares<PointType, PointNormalType> mls;
		//kdTreeモジュール
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

		mls.setComputeNormals(true);	//法線の計算を行うかどうか
		mls.setInputCloud(cloud);		//入力のポイントクラウド
		mls.setPolynomialFit(true);		//多項式フィッティングやるかどうか。オフにすると速くはなる。
		mls.setSearchMethod(tree);		//探索メソッドの設定
		mls.setSearchRadius(0.06);		//平滑化の具合（0.03位が打倒？0.06でだいぶ平らになる）http://www.cc.kyoto-su.ac.jp/~kano/pdf/study/student/2013YamamotoPresen.pdf
										//計算

		mls.process(*cloud_with_normals);

		return cloud_with_normals;
	}

	pcl::PolygonMesh PCLManager::createMeshWithOFM(pcl::PointCloud<PointType>::Ptr cloud)
	{
		//RangeImageに一度変換する
		float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
		float maxAngleWidth = (float)(120.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(60.0f * (M_PI / 180.0f));  // 180.0 degree in radians
		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.0;
		float minRange = 0.0f;
		int borderSize = 0;

		pcl::RangeImage rangeImage;
		rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		//std::cout << rangeImage << std::endl;

		//RangeImageから再度PointCloudにOrganized Point Cloudとして変換
		pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
		rangedCloud->width = rangeImage.width;
		rangedCloud->height = rangeImage.height;
		rangedCloud->resize(rangeImage.width * rangeImage.height);
		for (int i = 0; i < rangeImage.size(); i++)
		{
			pcl::copyPoint(rangeImage.points[i], rangedCloud->points[i]);
		}

		//メッシュ化
		pcl::OrganizedFastMesh<PointType> ofm;
		ofm.setTrianglePixelSize(2);

		ofm.setTriangulationType(pcl::OrganizedFastMesh<PointType>::TRIANGLE_ADAPTIVE_CUT);
		ofm.setInputCloud(rangedCloud);
		pcl::PolygonMesh mesh;
		ofm.reconstruct(mesh);

		return mesh;
	}

	pcl::PolygonMesh PCLManager::createMeshWithGP3(pcl::PointCloud<PointType>::Ptr cloud)
	{
		////法線の取得
		////法線格納用のスマートポインタを用意
		//pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalType>);
		////スムージング用のモジュールを用意
		//pcl::MovingLeastSquares<PointType, PointNormalType> mls;
		////kdTreeモジュール
		//pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);

		//mls.setComputeNormals(true);	//法線の計算を行うかどうか
		//mls.setInputCloud(cloud);		//入力のポイントクラウド
		//mls.setPolynomialFit(true);		//多項式フィッティングやるかどうか。オフにすると速くはなる。
		//mls.setSearchMethod(tree);		//探索メソッドの設定
		//mls.setSearchRadius(0.04);		//平滑化の具合（0.03位が打倒？0.06でだいぶ平らになる）http://www.cc.kyoto-su.ac.jp/~kano/pdf/study/student/2013YamamotoPresen.pdf
		//								//計算
		//mls.process(*cloud_with_normals);


		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(20);
		n.compute(*normals);
		//* normals should not contain the point normals + surface curvatures

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		//メッシュ化
		//Greedy Projection Triangulateアルゴリズムを使用
		pcl::GreedyProjectionTriangulation<PointNormalType> gp3;

		gp3.setSearchRadius(0.025);				//近傍探索に使うスフィアのサイズ
		gp3.setMu(2.5);							//近傍探索を行うのに使う乗数
		gp3.setMaximumNearestNeighbors(100);	//近傍にいくつ見つけるかの最大数設定

		gp3.setMaximumSurfaceAngle(M_PI);	//単位正規偏差がこれ以上だったら無視
		gp3.setMinimumAngle(M_PI / 18);			//各ポリゴンの持ちうる表面法線の傾き
		gp3.setMaximumAngle(2 * M_PI / 2);		//各ポリゴンの持ちうる表面法線の傾き最大値
		gp3.setNormalConsistency(true);		//（←要リサーチ）

											//結果の取得
		gp3.setInputCloud(cloud_with_normals);
		pcl::search::KdTree<PointNormalType>::Ptr tree3(new pcl::search::KdTree<PointNormalType>);
		gp3.setSearchMethod(tree3);
		pcl::PolygonMesh mesh;
		gp3.reconstruct(mesh);

		return mesh;
	}

	Eigen::Matrix4f PCLManager::iterativeClosestPoint(pcl::PointCloud<PointType>::Ptr target, pcl::PointCloud<PointType>::Ptr source)
	{
		pcl::IterativeClosestPoint<PointType, PointType> icp;
		icp.setInputTarget(target);
		icp.setInputSource(source);
		icp.align(*source);

		return icp.getFinalTransformation();
	}

	pcl::PolygonMesh PCLManager::concaveHull(pcl::PointCloud<PointType>::Ptr cloud) {
		// Create a Concave Hull representation of the projected inliers
		pcl::PointCloud<PointType>::Ptr cloud_hull(new pcl::PointCloud<PointType>);
		pcl::ConcaveHull<PointType> chull;
		chull.setInputCloud(cloud);
		chull.setAlpha(0.03);
		pcl::PointCloud<PointType>::Ptr voronoi_centers(new pcl::PointCloud<PointType>);
		chull.setVoronoiCenters(voronoi_centers);
		chull.setKeepInformation(true);
		vector<pcl::Vertices> polygons;
		chull.reconstruct(*cloud_hull, polygons);

		pcl::PolygonMesh mesh;

		pcl::toROSMsg(*cloud_hull, mesh.cloud);
		mesh.polygons = polygons;

		*cloud = *cloud_hull;


		return mesh;
	}

	void PCLManager::createRangeImage(pcl::PointCloud<PointType>::Ptr cloud, pcl::RangeImage &rangeImage)
	{
		//RangeImageに一度変換する
		float sensorAngleWidht = 360.f;
		float sensorAngleHeight = 180.f;
		float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
		float maxAngleWidth = (float)(sensorAngleWidht * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(sensorAngleHeight * (M_PI / 180.0f));  // 180.0 degree in radians
		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.0;
		float minRange = 0.0f;
		int borderSize = 0;

		rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		//std::cout << rangeImage << std::endl;
	}

	void PCLManager::createOrganizedCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
	{
		pcl::RangeImage rangeImage;
		createRangeImage(inputCloud, rangeImage);

		//RangeImageから再度PointCloudにOrganized Point Cloudとして変換
		pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
		rangedCloud->width = rangeImage.width;
		rangedCloud->height = rangeImage.height;
		rangedCloud->resize(rangeImage.width * rangeImage.height);
		rangedCloud->is_dense = false;
		pcl::copyPointCloud(rangeImage, *rangedCloud);
		*outputCloud = *rangedCloud;
	}

	void PCLManager::euclideanClusterExtraction(pcl::PointCloud<PointType>::Ptr cloud, vector<pcl::PointCloud<PointType>::Ptr> &outputCloud) {
		//pcl::gpu::DeviceArray<pcl::PointXYZ> gpuCloud;
		//gpuCloud.upload(cloud->points);
		//
		////Clusterに分割
		//pcl::gpu::Octree::Ptr gpuTree(new pcl::gpu::Octree);
		//gpuTree->setCloud(gpuCloud);

		vector<pcl::PointIndices> cluster_indices;
		//pcl::gpu::EuclideanClusterExtraction gpu_ec;
		//gpu_ec.setClusterTolerance(0.5); // 30cm
		//gpu_ec.setMinClusterSize(10);
		//gpu_ec.setMaxClusterSize(1000);
		//gpu_ec.setSearchMethod(gpuTree);
		//gpu_ec.setInput(gpuCloud);
		//gpu_ec.extract(cluster_indices);


		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);

		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.5); // 30cm
		ec.setMinClusterSize(10);
		ec.setMaxClusterSize(2000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);

		//分割されたインデックスを元に、クラウドを分割して返す
		//Reset
		for (int i = 0; i < outputCloud.size(); i++)
		{
			outputCloud[i].reset();
		}
		outputCloud.clear();

		//Thread
		boost::thread_group ths;
		for (int i = 0; i < cluster_indices.size(); i++)
		{
			pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
			outputCloud.push_back(p);
			ths.create_thread(boost::bind(&PCLManager::splitCloud, this, cloud, outputCloud[i], cluster_indices[i]));
		}
		ths.join_all();
	}

	void PCLManager::splitCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr ouputCloud, pcl::PointIndices &indices) {
		for (int j = 0; j < indices.indices.size(); j++)
		{
			int index = indices.indices[j];
			ouputCloud->push_back(inputCloud->points[index]);
		}
	}

	void PCLManager::createLineWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
	{
		//探索する近傍点の数
		int K = 10;

		//こいつに格納してく
		pcl::PointCloud<PointType>::Ptr c(new pcl::PointCloud<PointType>());

		outputCloud->clear();

		//kdTree
		pcl::KdTreeFLANN<PointType> kdTree;
		kdTree.setInputCloud(inputCloud);

		int count = 0;
		for (int i = 0; i < inputCloud->size(); i += 1)
		{
			//この点から最近傍の2点を取得して格納していく
			//探索対象の点
			PointType p = inputCloud->points[i];

			if (p.z < 2.0)
			{
				continue;
			}

			//結果が格納される配列
			vector<int> indexSearch(K);
			vector<float> distanceSearch(K);

			//探索
			if (kdTree.radiusSearch(p, 0.06, indexSearch, distanceSearch, 2))
			{
				for (int j = 1; j < indexSearch.size(); j += 1)
				{
					float distance = sqrtf(distanceSearch[j]);
					int index = indexSearch[j];
					//if (distance < 0.05)
					//{
					//	c->push_back(p);
					//	c->push_back(inputCloud->points[index]);
					//}
					c->push_back(p);
					c->push_back(inputCloud->points[index]);
				}
			}
		}


		/*cout << ";;;;;;;;;;;input: " << inputCloud->size() << ", " << c->size() << endl;*/
		//出力にマージ
		*outputCloud += *c;
	}


	void PCLManager::createPolygonWithRangeImage(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
	{
		//RangeImageに一度変換する
		float angularResolution = (float)(0.1f * (M_PI / 180.0f));  //   1.0 degree in radians
		float maxAngleWidth = (float)(120.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		float maxAngleHeight = (float)(60.0f * (M_PI / 180.0f));  // 180.0 degree in radians
		Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel = 0.0;
		float minRange = 0.0f;
		int borderSize = 0;

		pcl::RangeImage rangeImage;
		rangeImage.createFromPointCloud(*inputCloud, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
		//std::cout << rangeImage << std::endl;

		//RangeImageから再度PointCloudにOrganized Point Cloudとして変換
		pcl::PointCloud<PointType>::Ptr rangedCloud(new pcl::PointCloud<PointType>());
		rangedCloud->width = rangeImage.width;
		rangedCloud->height = rangeImage.height;
		rangedCloud->resize(rangeImage.width * rangeImage.height);
		for (int i = 0; i < rangeImage.size(); i++)
		{
			pcl::copyPoint(rangeImage.points[i], rangedCloud->points[i]);
		}

		outputCloud->clear();
		pcl::PointCloud<PointType>::Ptr triangleCloud(new pcl::PointCloud<PointType>());
		for (int y = 1; y < rangedCloud->height - 1; y++)
		{
			for (int x = 1; x < rangedCloud->width - 1; x++)
			{
				int index = y * rangedCloud->width + x;
				PointType p = rangedCloud->points[index];


				//左
				int indexLeft = y * rangedCloud->width + (x - 1);
				PointType leftP = rangedCloud->points[indexLeft];
				//上
				int indexTop = (y - 1) * rangedCloud->width + x;
				PointType topP = rangedCloud->points[indexTop];
				//右
				int indexRight = y * rangedCloud->width + (x + 1);
				PointType rightP = rangedCloud->points[indexRight];
				//下
				int indexBtm = (y + 1) * rangedCloud->width + x;
				PointType btmP = rangedCloud->points[indexBtm];

				//左上ポリゴン
				if (p.z > 1 && leftP.z > 1 && topP.z > 1)
				{
					triangleCloud->points.push_back(p);
					triangleCloud->points.push_back(topP);
					triangleCloud->points.push_back(leftP);
				}

				////右下ポリゴン
				//if (rightP.z > 1 && btmP.z > 1)
				//{
				//	triangleCloud->points.push_back(p);
				//	triangleCloud->points.push_back(btmP);
				//	triangleCloud->points.push_back(rightP);
				//}

				////左下ポリゴン
				//if (leftP.z > 1 && btmP.z > 1)
				//{
				//	triangleCloud->points.push_back(p);
				//	triangleCloud->points.push_back(leftP);
				//	triangleCloud->points.push_back(btmP);
				//}

				//右上ポリゴン
				if (p.z > 1 && rightP.z > 1 && topP.z > 1)
				{
					triangleCloud->points.push_back(p);
					triangleCloud->points.push_back(rightP);
					triangleCloud->points.push_back(topP);
				}
			}
		}

		*outputCloud = *triangleCloud;
	}


	void PCLManager::createPolygonWithCloud(pcl::PointCloud<PointType>::Ptr inputCloud, pcl::PointCloud<PointType>::Ptr outputCloud)
	{
		//探索する近傍点の数
		int K = 90;

		//kdTree
		pcl::KdTreeFLANN<PointType> kdTree;
		kdTree.setInputCloud(inputCloud);


		for (int i = 0; i < inputCloud->size(); i += 1)
		{
			//この点から最近傍の2点を取得して格納していく
			//探索対象の点
			PointType p = inputCloud->points[i];

			//結果が格納される配列
			vector<int> indexSearch(K);
			vector<float> distanceSearch(K);

			//探索
			if (kdTree.radiusSearch(p, 0.3, indexSearch, distanceSearch, K))
			{
				if (indexSearch.size() >= 3)
				{
					for (int j = 0; j < indexSearch.size(); j += 3)
					{
						if (sqrtf(distanceSearch[j]) + sqrtf(distanceSearch[j + 1]) + sqrtf(distanceSearch[j + 2]) < 0.36)
						{
							outputCloud->push_back(inputCloud->points[indexSearch[j]]);
							outputCloud->push_back(inputCloud->points[indexSearch[j + 1]]);
							outputCloud->push_back(inputCloud->points[indexSearch[j + 2]]);
						}
					}
				}
			}
		}
	}

	pcl::PointCloud<PointType>::Ptr PCLManager::projectionToZ(pcl::PointCloud<PointType>::Ptr cloud, float zValue) {
		//与えられたZの値にある平面にクラウドをプロジェクションする
		// Create a set of planar coefficients with X=Y=0,Z=1
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.resize(4);
		coefficients->values[0] = coefficients->values[1] = 0;
		coefficients->values[2] = 1.0;
		coefficients->values[3] = zValue;

		// Create the filtering object
		pcl::PointCloud<PointType>::Ptr cloud_projected(new pcl::PointCloud<PointType>());
		pcl::ProjectInliers<PointType> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.filter(*cloud_projected);

		return cloud_projected;
	}

	Eigen::Vector4f PCLManager::centroid(pcl::PointCloud<PointType>::Ptr cloud)
	{
		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*cloud, xyz_centroid);//重心を計算

		return xyz_centroid;
	}

	void PCLManager::movingLeastSquares(pcl::PointCloud<PointType>::Ptr inputCloud)
	{
		// Testing upsampling
		pcl::PointCloud<PointNormalType>::Ptr mls_normals(new pcl::PointCloud<PointNormalType>());
		pcl::MovingLeastSquares<PointType, PointNormalType> mls_upsampling;
		// Set parameters
		mls_upsampling.setInputCloud(inputCloud);
		mls_upsampling.setComputeNormals(true);
		mls_upsampling.setPolynomialFit(true);
		pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
		mls_upsampling.setSearchMethod(tree);
		mls_upsampling.setSearchRadius(0.06);
		//mls_upsampling.setUpsamplingMethod(pcl::MovingLeastSquares<PointType, PointNormalType>::SAMPLE_LOCAL_PLANE);
		//mls_upsampling.setUpsamplingRadius(0.005);
		//mls_upsampling.setUpsamplingStepSize(0.005);

		mls_normals->clear();
		mls_upsampling.process(*mls_normals);

		pcl::copyPointCloud(*mls_normals, *inputCloud);
	}


	void PCLManager::detect_keypoints(const pcl::PointCloud<PointType>::ConstPtr & cloud) {
		pcl::HarrisKeypoint2D<PointType, pcl::PointXYZI> harris;
		harris.setInputCloud(cloud);
		harris.setNumberOfThreads(6);
		harris.setNonMaxSupression(true);
		harris.setRadiusSearch(0.01);
		harris.setMethod(pcl::HarrisKeypoint2D<PointType, pcl::PointXYZI>::TOMASI);
		harris.setThreshold(0.05);
		harris.setWindowWidth(5);
		harris.setWindowHeight(5);
		pcl::PointCloud<pcl::PointXYZI>::Ptr response(new pcl::PointCloud<pcl::PointXYZI>);
		harris.compute(*response);

		feature_points = harris.getKeypointsIndices();
	}
}