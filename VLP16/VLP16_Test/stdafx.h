// stdafx.h : 標準のシステム インクルード ファイルのインクルード ファイル、または
// 参照回数が多く、かつあまり変更されない、プロジェクト専用のインクルード ファイル
// を記述します。
//

#pragma once

#include "targetver.h"

//---------------------------------
//	C++11
//---------------------------------
#define _USE_MATH_DEFINES // for C++
#include <thread>

//---------------------------------
//	Boost
//---------------------------------
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/property_tree/xml_parser.hpp>


//---------------------------------
//	PCL
//---------------------------------
//IO
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//hdl_grabber
#include <pcl/io/hdl_grabber.h>
//Visualization
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
//Filter
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
//Surface
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
//Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
//ROS
#include <pcl/ros/conversions.h>
//Registration
#include <pcl/registration/icp.h>
//KD Tree
#include <pcl/kdtree/kdtree_flann.h>
//Feature
#include <pcl/features/normal_3d.h>
//Image
#include <pcl/range_image/range_image.h>
//VLP Grabber
//#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
//Console
#include <pcl/console/parse.h>
//Tracker
#include <pcl/tracking/pyramidal_klt.h>
#include <pcl/keypoints/harris_2d.h>


#include <omp.h>

//---------------------------------
//	OSC
//---------------------------------
#include "OscReceivedElements.h"
#include "OscPacketListener.h"
#include "OscOutboundPacketStream.h"
#include "UdpSocket.h"
#include "IpEndpointName.h"