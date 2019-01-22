#pragma once

////UE4‚©‚çŽg‚¤—p
//#if defined VLP16DLL_EXPORT
//#define VLP16DLL_API __declspec(dllexport)
//#else
//#define VLP16DLL_API __declspec(dllimport)
//#endif

#include "Sensor.h"
#include "PCLManager.h"

namespace VLP16 {
	class VLP16
	{
	public:
		VLP16();
		~VLP16();

		//Interface
		void Start(string pcapName);
		void Start(string ipAddress, unsigned short port);
		void Close();
		void Run();

		void SetSensorPosture(double x, double y, double z, double pitch, double yaw, double roll);
		void SetRange(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
	private:
		boost::shared_ptr<pcl::VLPGrabber> mVlpGrabber;
		boost::signals2::connection mConnection;
		pcl::PointCloud<PointType>::ConstPtr mCloud;
		boost::mutex mVLPMutex;
		bool mRunning;
		Sensor &mSensor = Sensor::GetInstance();

		void Start();
		void vlpCallback(const pcl::PointCloud<PointType>::ConstPtr& cloudPtr);

		//Centroid
		int numMaxCentroids = 100;
		vector<Eigen::Vector4f> curCentroids;
		void calcTrackingCentroids(vector<Eigen::Vector4f> &inputCentroids);
	};
}
