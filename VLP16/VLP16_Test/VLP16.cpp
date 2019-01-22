// VLP16.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "VLP16.h"
#include "OSCSender.h"

namespace VLP16 {
	VLP16::VLP16():
		mRunning(false)
	{
	}

	VLP16::~VLP16()
	{
		Close();
	}

	void VLP16::Start(string pcapName)
	{
		mVlpGrabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(pcapName));
		Start();
	}

	void VLP16::Start(string ipAddress, unsigned short port)
	{
		mVlpGrabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(ipAddress), boost::lexical_cast<unsigned short>(port)));
		Start();
	}

	void VLP16::Close()
	{
		mRunning = false;
		mVlpGrabber->stop();
		mConnection.disconnect();

		pcl::PointCloud<PointType> yea;
	}

	void VLP16::Start()
	{
		boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> cb = boost::bind(&VLP16::vlpCallback, this, _1);
		mConnection = mVlpGrabber->registerCallback(cb);
		mVlpGrabber->start();
	}

	void VLP16::Run()
	{
		mRunning = true;
		pcl_func::PCLManager pcl;

		//Sensor setup
		mSensor.loadSensorRangeData();
		mSensor.loadSensorPostureData();

		curCentroids.clear();
		while (mRunning)
		{
			if (mCloud) {
				//Update Cloud
				mVLPMutex.lock();
				pcl::PointCloud<PointType>::Ptr p(new pcl::PointCloud<PointType>());
				*p = *mCloud;
				//cout << mCloud->points.size() << endl;
				mVLPMutex.unlock();
				
				//PCL
				pcl.update(mSensor, p);

				//Centroids
				vector<Eigen::Vector4f> centroids = pcl.getCentroids();
				//calcTrackingCentroids(centroids);

				cout << "===================" << endl;
				if (centroids.size() == 0)
				{
					OSCSender::GetInstance().sendOSCMessage(0, 0, 0, 0, 0);
				}
				for (int i = 0; i < centroids.size(); i++)
				{
					if (centroids[i].x() < -100) continue;
					cout << i << ": (" << centroids[i].x() << ", " << centroids[i].y() << ", " << centroids[i].z() << ")" << endl;
					
					//Send OSC
					OSCSender::GetInstance().sendOSCMessage(centroids.size(), i, centroids[i].x(), centroids[i].y(), centroids[i].z());
				}
				cout << endl;
			}
			boost::this_thread::sleep(boost::posix_time::microseconds(1));
		}
		Close();
	}

	void VLP16::SetSensorPosture(double x, double y, double z, double pitch, double yaw, double roll)
	{
		mSensor.setPopsture(x, y, z, pitch, yaw, roll);
	}

	void VLP16::SetRange(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
	{
		mSensor.setRange(minX, maxX, minY, maxY, minZ, maxZ);
	}

	//Callback
	void VLP16::vlpCallback(const pcl::PointCloud<PointType>::ConstPtr& cloudPtr)
	{
		boost::mutex::scoped_lock lock(mVLPMutex);
		mCloud = cloudPtr;
	}


	void VLP16::calcTrackingCentroids(vector<Eigen::Vector4f> &inputCentroids) {
		if (inputCentroids.empty())
		{
			return;
		}
		numMaxCentroids = 100;
		if (inputCentroids.size() > numMaxCentroids)
		{
			inputCentroids.resize(numMaxCentroids);
		}

		//一番最初は空なので、そのままいれる
		if (curCentroids.empty())
		{
			curCentroids.resize(100);
			for (int i = 0; i < curCentroids.size(); i++)
			{
				curCentroids[i].x() = -999;
				curCentroids[i].y() = -999;
				curCentroids[i].z() = -999;
			}
			return;
		}

		//それ以外は、現状のものに近しいものを採用していく
		//現状のものを基準に走査していき、該当しそうなものがない場合は削除
		//kdTreeのRadius Searchで一定範囲内のポイントを検索し、該当するものがあれば採用
		//なければそのインデックスを空ける
		//新規で何か出現した場合は、近傍でないインデックスのやつがそれっぽいので近傍で引っかかったやつをストックしておいてそれ以外を採用
		vector<int> usedIndices(0);

		//まず重心点たちをポイントクラウドに変換
		pcl::PointCloud<pcl::PointXYZ>::Ptr centroidCloud(new pcl::PointCloud<pcl::PointXYZ>());
		for (int i = 0; i < inputCentroids.size(); i++)
		{
			pcl::PointXYZ p;
			p.x = inputCentroids[i].x();
			p.y = inputCentroids[i].y();
			p.z = inputCentroids[i].z();

			centroidCloud->points.push_back(p);
		}
		//kdTree
		pcl::search::KdTree<pcl::PointXYZ> tree;
		tree.setInputCloud(centroidCloud);
		float radius = 0.1f;
		for (int i = 0; i < curCentroids.size(); i++)
		{
			pcl::PointXYZ searchP;
			searchP.x = curCentroids[i].x();
			searchP.y = curCentroids[i].y();
			searchP.z = curCentroids[i].z();
			//
			//if (searchP.x < -100.f)
			//{
			//	continue;
			//}


			vector<int> pointIdxRadiusSearch;
			vector<float> pointRadiusSquaredDistance;

			int result = tree.radiusSearch(searchP, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
			if (result > 0)
			{
				//一番距離が近い奴を採用
				//とりあえず一番距離が近い奴をピックアップ
				int nearestIndex = 0;
				float dist = 100000;
				for (int j = 0; j < pointRadiusSquaredDistance.size(); j++)
				{
					int index = pointIdxRadiusSearch[j];
					cout << "Point: " << i << ": (" << searchP.x << ", " << searchP.y << ", " << searchP.z << ")" << "---"
						<< pointIdxRadiusSearch[j] << ": (" << inputCentroids[index].x() << ", " << inputCentroids[index].y() << ", " << inputCentroids[index].z() << ")" << endl;
					if (pointRadiusSquaredDistance[j] < dist)
					{
						dist = pointRadiusSquaredDistance[j];
						nearestIndex = pointIdxRadiusSearch[j];
					}
				}
				
				//index = i の点に関しては、nearestIndexの点の情報を採用
				curCentroids[i] = inputCentroids[pointIdxRadiusSearch[0]];

				//使ったやつはストック
				usedIndices.push_back(pointIdxRadiusSearch[0]);
			}
			else {
				//そもそも検出されなかった場合は、そのインデックスを不正値で埋めとく
				curCentroids[i].x() = -999.f;
				curCentroids[i].y() = -999.f;
				curCentroids[i].z() = -999.f;
			}
		}

		//新規っぽいポイントたちを採用していく
		for (int i = 0; i < inputCentroids.size(); i++)
		{
			bool used = false;
			
			for (int j = 0; j < usedIndices.size(); j++)
			{
				if (i == usedIndices[j])
				{
					used = true;
					//break;
				}
			}
			
			
			//未使用だったら、空いてそうなインデックスにぶっこむ
			if (!used)
			{
				cout << "Unused: " << ": (" << inputCentroids[i].x() << ", " << inputCentroids[i].y() << ", " << inputCentroids[i].z() << ")" << endl;
				for (int j = 0; j < curCentroids.size(); j++)
				{
					if (curCentroids[j].x() < -100.f)
					{
						curCentroids[j] = inputCentroids[i];
						usedIndices.push_back(i);
						break;
					}
				}
			}
		}
	}
}
