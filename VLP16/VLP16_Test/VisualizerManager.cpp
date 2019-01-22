#include "stdafx.h"
#include "Sensor.h"
#include "VisualizerManager.h"

using namespace std;

VisualizerManager::VisualizerManager()
{
	showCloud.reset(new pcl::PointCloud<PointType>());
	viewer.reset(new pcl::visualization::PCLVisualizer("VLP16 Viewer"));
	setupVisualizer(showCloud);
	editMode = Edit_Mode_None;
	updateDebugInfo();
}

void VisualizerManager::updateVisualizer(pcl::PointCloud<PointType>::Ptr inputCloud)
{
	mutex_lock.lock();
	*showCloud = *inputCloud;
	mutex_lock.unlock();

	for (int i = 0; i < 4; i++)
	{
		string idStr = "Sensor v" + to_string(i + 1);
		viewer->updatePointCloud(showCloud, idStr);
	}
	viewer->spinOnce();
}

void VisualizerManager::setupVisualizer(pcl::PointCloud<PointType>::Ptr inputCloud) {
	//-----V1 bottom-left
	int v1(0);
	viewer->createViewPort(0, 0, 0.5, 0.5, v1);
	viewer->addPointCloud(inputCloud, "Sensor v1", v1);
	viewer->createViewPortCamera(v1);

	//-----V2 bottom-right
	int v2(0);
	viewer->createViewPort(0.5, 0, 1.0, 0.5, v2);
	viewer->addText("Top", 10, 10, "v2", v2);
	viewer->addPointCloud(inputCloud, "Sensor v2", v2);
	viewer->createViewPortCamera(v2);

	//-----V3 top-left
	int v3(0);
	viewer->createViewPort(0, 0.5, 0.5, 1.0, v3);
	viewer->addText("Front", 10, 10, "v3", v3);
	viewer->addPointCloud(inputCloud, "Sensor v3", v3);
	viewer->createViewPortCamera(v3);

	//-----V4 top-right
	int v4(0);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v4);
	viewer->addText("Side", 10, 10, "v4", v4);
	viewer->addPointCloud(inputCloud, "Sensor v4", v4);
	viewer->createViewPortCamera(v4);

	//View Points
	setDefaultViewPoints();

	//-----Common
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5);
	viewer->addCoordinateSystem(0.5);
	viewer->registerKeyboardCallback(&VisualizerManager::keyboardEventOccurred, *this, (void*)&viewer);
}

void VisualizerManager::setDefaultViewPoints() {
	//View Points
	viewer->initCameraParameters();
	viewer->setCameraPosition(3.0, 3.0, -3.0, 0, 0, 0, 1);	//3D
	viewer->setCameraPosition(0, 0.0, 6.0, 0, 1, 0, 2);	//Top
	viewer->setCameraPosition(0, -3.0, 0.0, 0, 0, 1, 3);	//Front
	viewer->setCameraPosition(-4.0, 0.0, 0.0, -1, 0, 0, 4);	//Side
}

void VisualizerManager::updateDebugInfo() {
	//-----Edit Mode
	string editModeStr;
	switch (editMode)
	{
	case Edit_Mode_None:
		editModeStr = " Edit mode -> 1: X, 2: Y, 3: Z, 4: Pitch, 5: Yaw, 6: Roll\n Command -> s: save, l: load, 0: reset view";
		break;
	case Edit_Mode_X:
		editModeStr = "Edit Mode ----- X /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Y:
		editModeStr = "Edit Mode ----- Y /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Z:
		editModeStr = "Edit Mode ----- Z /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Pitch:
		editModeStr = "Edit Mode ----- Pitch /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Yaw:
		editModeStr = "Edit Mode ----- Yaw /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Roll:
		editModeStr = "Edit Mode ----- Roll /// (Up key: increase, Down key: decrease)";
		break;
	case Edit_Mode_Save:
		editModeStr = "Edit Mode ----- Saved data to Kinect_Settings.xml";
		break;
	case Edit_Mode_Load:
		editModeStr = "Edit Mode ----- Loaded data from Kinect_Settings.xml";
		break;
	default:
		editModeStr = "";
		break;
	}

	viewer->removeText3D("v1", 1);
	viewer->addText(editModeStr, 10, 10, "v1", 1);
}


void VisualizerManager::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	cout << event.getKeySym() << endl;
	string inputKey = event.getKeySym();

	Sensor &sensor = Sensor::GetInstance();

	//---------- Reset View
	if (inputKey == "0")
	{
		setDefaultViewPoints();
	}
	//-----Save Settings
	if (inputKey == "s" || inputKey == "S")
	{
		sensor.saveSensorPostureData();
		editMode = Edit_Mode_Save;
	}
	//-----Load Settings
	if (inputKey == "l" || inputKey == "L")
	{
		sensor.loadSensorPostureData();
		editMode = Edit_Mode_Load;
	}
	//-----Change Edit Mode
	if (inputKey == "1")
	{
		editMode = Edit_Mode_X;
	}
	else if (inputKey == "2")
	{
		editMode = Edit_Mode_Y;
	}
	else if (inputKey == "3")
	{
		editMode = Edit_Mode_Z;
	}
	else if (inputKey == "4")
	{
		editMode = Edit_Mode_Pitch;
	}
	else if (inputKey == "5")
	{
		editMode = Edit_Mode_Yaw;
	}
	else if (inputKey == "6")
	{
		editMode = Edit_Mode_Roll;
	}
	else if (inputKey == "Down" || inputKey == "Up") {
		//do nothing
	}
	else {
		editMode = Edit_Mode_None;
	}


	switch (editMode)
	{
	case Edit_Mode_X:
		if (inputKey == "Down")
		{
			sensor.setX(sensor.getX() - 0.01);
		}
		if (inputKey == "Up")
		{
			sensor.setX(sensor.getX() + 0.01);
		}
		break;
	case Edit_Mode_Y:
		if (inputKey == "Down")
		{
			sensor.setY(sensor.getY() - 0.01);
		}
		if (inputKey == "Up")
		{
			sensor.setY(sensor.getY() + 0.01);
		}
		break;
	case Edit_Mode_Z:
		if (inputKey == "Down")
		{
			sensor.setZ(sensor.getZ() - 0.01);
		}
		if (inputKey == "Up")
		{
			sensor.setZ(sensor.getZ() + 0.01);
		}
		break;
	case Edit_Mode_Pitch:
		if (inputKey == "Down")
		{
			sensor.setPitch(sensor.getPitch() - 0.1);
		}
		if (inputKey == "Up")
		{
			sensor.setPitch(sensor.getPitch() + 0.1);
		}
		break;
	case Edit_Mode_Yaw:
		if (inputKey == "Down")
		{
			sensor.setYaw(sensor.getYaw() - 0.1);
		}
		if (inputKey == "Up")
		{
			sensor.setYaw(sensor.getYaw() + 0.1);
		}
		break;
	case Edit_Mode_Roll:
		if (inputKey == "Down")
		{
			sensor.setRoll(sensor.getRoll() - 0.1);
		}
		if (inputKey == "Up")
		{
			sensor.setRoll(sensor.getRoll() + 0.1);
		}
		break;
	default:
		break;
	}

	updateDebugInfo();
	cout << inputKey << endl;
}

void VisualizerManager::mouseEventOccurred(const pcl::visualization::KeyboardEvent & event, void * viewer_void)
{
}