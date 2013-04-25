#include "stdafx.h"
#include "display.h"
#include "debug.h"

using namespace std;

Display::Display(void)
{
}


Display::~Display(void)
{
}


void Display::run(PointCloudCPtr cloud)
{
	DEBUG("starting viewer" << endl);
	pcl::visualization::CloudViewer viewer("PCD Viewier");
	viewer.showCloud(cloud);
	
	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	DEBUG("viewer exited" << endl);
}