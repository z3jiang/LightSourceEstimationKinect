#pragma once

#include "stdafx.h"

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}


	void run ();

private:
	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	pcl::visualization::CloudViewer viewer;

};
