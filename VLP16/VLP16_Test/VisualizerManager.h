#pragma once


enum Edit_Mode {
	Edit_Mode_None = 0,
	Edit_Mode_X,
	Edit_Mode_Y,
	Edit_Mode_Z,
	Edit_Mode_Pitch,
	Edit_Mode_Yaw,
	Edit_Mode_Roll,
	Edit_Mode_Save,
	Edit_Mode_Load
};

class VisualizerManager
{
public:
	VisualizerManager();
	~VisualizerManager() {}

	void updateVisualizer(pcl::PointCloud<PointType>::Ptr inputCloud);

	

private:
	//Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::PointCloud<PointType>::Ptr showCloud;
	void setupVisualizer(pcl::PointCloud<PointType>::Ptr inputCloud);
	void setDefaultViewPoints();
	void updateDebugInfo();
	Edit_Mode editMode;

	//Event
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent & event, void * viewer_void);
	void mouseEventOccurred(const pcl::visualization::KeyboardEvent & event, void * viewer_void);

	boost::mutex mutex_lock;
};

